#include "Limelight-internal.h"

#define FIRST_FRAME_MAX 1500
#define FIRST_FRAME_TIMEOUT_SEC 10

#define FIRST_FRAME_PORT 47996

// We can't request an IDR frame until the depacketizer knows
// that a packet was lost. This timeout bounds the time that
// the RTP queue will wait for missing/reordered packets.
#define RTP_QUEUE_DELAY 10

// This is the desired number of video packets that can be
// stored in the socket's receive buffer. 2048 is chosen
// because it should be large enough for all reasonable
// frame sizes (probably 2 or 3 frames) without using too
// much kernel memory with larger packet sizes. It also
// can smooth over transient pauses in network traffic
// and subsequent packet/frame bursts that follow.
#define RTP_RECV_PACKETS_BUFFERED 2048

// Per-stream state for each independent video stream
typedef struct _VIDEO_STREAM_STATE {
    RTP_VIDEO_QUEUE rtpQueue;

    SOCKET rtpSocket;
    SOCKET firstFrameSocket;

    PPLT_CRYPTO_CONTEXT decryptionCtx;

    PLT_THREAD udpPingThread;
    PLT_THREAD receiveThread;
    PLT_THREAD decoderThread;

    bool receivedDataFromPeer;
    uint64_t firstDataTimeMs;
    bool receivedFullFrame;

    int streamIndex;
} VIDEO_STREAM_STATE;

static VIDEO_STREAM_STATE videoStreams[MAX_VIDEO_STREAMS];
static int numActiveVideoStreams;

// Initialize the video stream(s)
void initializeVideoStream(void) {
    numActiveVideoStreams = NumVideoStreams;

    for (int i = 0; i < numActiveVideoStreams; i++) {
        VIDEO_STREAM_STATE* state = &videoStreams[i];

        initializeVideoDepacketizer(i, StreamConfig.packetSize);
        RtpvInitializeQueue(&state->rtpQueue);
        state->rtpQueue.streamIndex = i;
        state->decryptionCtx = PltCreateCryptoContext();
        state->receivedDataFromPeer = false;
        state->firstDataTimeMs = 0;
        state->receivedFullFrame = false;
        state->rtpSocket = INVALID_SOCKET;
        state->firstFrameSocket = INVALID_SOCKET;
        state->streamIndex = i;
    }
}

// Clean up the video stream(s)
void destroyVideoStream(void) {
    for (int i = 0; i < numActiveVideoStreams; i++) {
        VIDEO_STREAM_STATE* state = &videoStreams[i];

        PltDestroyCryptoContext(state->decryptionCtx);
        RtpvCleanupQueue(&state->rtpQueue);
    }

    for (int i = 0; i < numActiveVideoStreams; i++) {
        destroyVideoDepacketizer(i);
    }
}

// UDP Ping proc -- per-stream
static void VideoPingThreadProc(void* context) {
    VIDEO_STREAM_STATE* state = (VIDEO_STREAM_STATE*)context;
    char legacyPingData[] = { 0x50, 0x49, 0x4E, 0x47 };
    LC_SOCKADDR saddr;

    LC_ASSERT(VideoPortNumbers[state->streamIndex] != 0);

    memcpy(&saddr, &RemoteAddr, sizeof(saddr));
    SET_PORT(&saddr, VideoPortNumbers[state->streamIndex]);

    // We do not check for errors here. Socket errors will be handled
    // on the read-side in ReceiveThreadProc(). This avoids potential
    // issues related to receiving ICMP port unreachable messages due
    // to sending a packet prior to the host PC binding to that port.
    int pingCount = 0;
    while (!PltIsThreadInterrupted(&state->udpPingThread)) {
        if (VideoPingPayloads[state->streamIndex].payload[0] != 0) {
            pingCount++;
            VideoPingPayloads[state->streamIndex].sequenceNumber = BE32(pingCount);

            sendto(state->rtpSocket, (char*)&VideoPingPayloads[state->streamIndex],
                   sizeof(VideoPingPayloads[state->streamIndex]), 0,
                   (struct sockaddr*)&saddr, AddrLen);
        }
        else {
            sendto(state->rtpSocket, legacyPingData, sizeof(legacyPingData), 0,
                   (struct sockaddr*)&saddr, AddrLen);
        }

        PltSleepMsInterruptible(&state->udpPingThread, 500);
    }
}

// Receive thread proc -- per-stream
static void VideoReceiveThreadProc(void* context) {
    VIDEO_STREAM_STATE* state = (VIDEO_STREAM_STATE*)context;
    int err;
    int bufferSize, receiveSize, decryptedSize, minSize;
    char* buffer;
    char* encryptedBuffer;
    int queueStatus;
    bool useSelect;
    int waitingForVideoMs;
    bool encrypted;

    encrypted = !!(EncryptionFeaturesEnabled & SS_ENC_VIDEO);
    decryptedSize = StreamConfig.packetSize + MAX_RTP_HEADER_SIZE;
    minSize = sizeof(RTP_PACKET) + ((EncryptionFeaturesEnabled & SS_ENC_VIDEO) ? sizeof(ENC_VIDEO_HEADER) : 0);
    receiveSize = decryptedSize + ((EncryptionFeaturesEnabled & SS_ENC_VIDEO) ? sizeof(ENC_VIDEO_HEADER) : 0);
    bufferSize = decryptedSize + sizeof(RTPV_QUEUE_ENTRY);
    buffer = NULL;

    if (setNonFatalRecvTimeoutMs(state->rtpSocket, UDP_RECV_POLL_TIMEOUT_MS) < 0) {
        // SO_RCVTIMEO failed, so use select() to wait
        useSelect = true;
    }
    else {
        // SO_RCVTIMEO timeout set for recv()
        useSelect = false;
    }

    // Allocate a staging buffer to use for each received packet
    if (encrypted) {
        encryptedBuffer = (char*)malloc(receiveSize);
        if (encryptedBuffer == NULL) {
            Limelog("Video Receive [%d]: malloc() failed\n", state->streamIndex);
            ListenerCallbacks.connectionTerminated(-1);
            return;
        }
    }
    else {
        encryptedBuffer = NULL;
    }

    waitingForVideoMs = 0;
    while (!PltIsThreadInterrupted(&state->receiveThread)) {
        PRTP_PACKET packet;

        if (buffer == NULL) {
            buffer = (char*)malloc(bufferSize);
            if (buffer == NULL) {
                Limelog("Video Receive [%d]: malloc() failed\n", state->streamIndex);
                ListenerCallbacks.connectionTerminated(-1);
                break;
            }
        }

        err = recvUdpSocket(state->rtpSocket,
                            encrypted ? encryptedBuffer : buffer,
                            receiveSize,
                            useSelect);
        if (err < 0) {
            Limelog("Video Receive [%d]: recvUdpSocket() failed: %d\n", state->streamIndex, (int)LastSocketError());
            ListenerCallbacks.connectionTerminated(LastSocketFail());
            break;
        }
        else if  (err == 0) {
            if (!state->receivedDataFromPeer) {
                // If we wait many seconds without ever receiving a video packet,
                // assume something is broken and terminate the connection.
                waitingForVideoMs += UDP_RECV_POLL_TIMEOUT_MS;
                if (waitingForVideoMs >= FIRST_FRAME_TIMEOUT_SEC * 1000) {
                    Limelog("Terminating connection due to lack of video traffic on stream %d\n", state->streamIndex);
                    ListenerCallbacks.connectionTerminated(ML_ERROR_NO_VIDEO_TRAFFIC);
                    break;
                }
            }

            // Receive timed out; try again
            continue;
        }

        if (!state->receivedDataFromPeer) {
            state->receivedDataFromPeer = true;
            Limelog("Received first video packet on stream %d after %d ms\n", state->streamIndex, waitingForVideoMs);

            state->firstDataTimeMs = PltGetMillis();
        }

#ifndef LC_FUZZING
        if (!state->receivedFullFrame) {
            if (PltGetMillis() - state->firstDataTimeMs >= FIRST_FRAME_TIMEOUT_SEC * 1000) {
                Limelog("Terminating connection due to lack of a successful video frame on stream %d\n", state->streamIndex);
                ListenerCallbacks.connectionTerminated(ML_ERROR_NO_VIDEO_FRAME);
                break;
            }
        }
#endif

        if (err < minSize) {
            // Runt packet
            continue;
        }

        // Decrypt the packet into the buffer if encryption is enabled
        if (encrypted) {
            PENC_VIDEO_HEADER encHeader = (PENC_VIDEO_HEADER)encryptedBuffer;

            // If this frame is below our current frame number, discard it before decryption
            // to save CPU cycles decrypting FEC shards for a frame we already reassembled.
            //
            // Since this is happening _before_ decryption, this packet is not trusted yet.
            // It's imperative that we do not mutate any state based on this packet until
            // after it has been decrypted successfully!
            if (encHeader->frameNumber && LE32(encHeader->frameNumber) < RtpvGetCurrentFrameNumber(&state->rtpQueue)) {
                continue;
            }

            if (!PltDecryptMessage(state->decryptionCtx, ALGORITHM_AES_GCM, 0,
                                   (unsigned char*)StreamConfig.remoteInputAesKey, sizeof(StreamConfig.remoteInputAesKey),
                                   encHeader->iv, sizeof(encHeader->iv),
                                   encHeader->tag, sizeof(encHeader->tag),
                                   ((unsigned char*)(encHeader + 1)), err - sizeof(ENC_VIDEO_HEADER),
                                   (unsigned char*)buffer, &err)) {
                Limelog("Failed to decrypt video packet on stream %d!\n", state->streamIndex);
                continue;
            }
        }

        // Convert fields to host byte-order
        packet = (PRTP_PACKET)&buffer[0];
        packet->sequenceNumber = BE16(packet->sequenceNumber);
        packet->timestamp = BE32(packet->timestamp);
        packet->ssrc = BE32(packet->ssrc);

        queueStatus = RtpvAddPacket(&state->rtpQueue, packet, err, (PRTPV_QUEUE_ENTRY)&buffer[decryptedSize]);

        if (queueStatus == RTPF_RET_QUEUED) {
            // The queue owns the buffer
            buffer = NULL;
        }
    }

    if (buffer != NULL) {
        free(buffer);
    }

    if (encryptedBuffer != NULL) {
        free(encryptedBuffer);
    }
}

void notifyKeyFrameReceived(void) {
    // Remember that we got a full frame successfully on stream 0
    // For multi-stream, this is called from the depacketizer which
    // currently operates on the primary stream's queue
    videoStreams[0].receivedFullFrame = true;
}

// Decoder thread proc (used for non-direct-submit decoders)
static void VideoDecoderThreadProc(void* context) {
    VIDEO_STREAM_STATE* state = (VIDEO_STREAM_STATE*)context;

    while (!PltIsThreadInterrupted(&state->decoderThread)) {
        VIDEO_FRAME_HANDLE frameHandle;
        PDECODE_UNIT decodeUnit;

        if (!LiWaitForNextVideoFrame(&frameHandle, &decodeUnit)) {
            return;
        }

        LiCompleteVideoFrame(frameHandle, VideoCallbacks.submitDecodeUnit(decodeUnit));
    }
}

// Read the first frame of the video stream (Gen 3 only)
static int readFirstFrame(VIDEO_STREAM_STATE* state) {
    // All that matters is that we close this socket.
    // This starts the flow of video on Gen 3 servers.

    closeSocket(state->firstFrameSocket);
    state->firstFrameSocket = INVALID_SOCKET;

    return 0;
}

// Terminate the video stream(s)
void stopVideoStream(void) {
    bool anyReceivedData = false;

    for (int i = 0; i < numActiveVideoStreams; i++) {
        if (videoStreams[i].receivedDataFromPeer) {
            anyReceivedData = true;
            break;
        }
    }

    if (!anyReceivedData) {
        Limelog("No video traffic was ever received from the host!\n");
    }

    VideoCallbacks.stop();

    // Wake up client code that may be waiting on the decode unit queue
    for (int i = 0; i < numActiveVideoStreams; i++) {
        stopVideoDepacketizer(i);
    }

    // Interrupt all threads for all streams
    for (int i = 0; i < numActiveVideoStreams; i++) {
        VIDEO_STREAM_STATE* state = &videoStreams[i];

        PltInterruptThread(&state->udpPingThread);
        PltInterruptThread(&state->receiveThread);
        if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
            PltInterruptThread(&state->decoderThread);
        }

        if (state->firstFrameSocket != INVALID_SOCKET) {
            shutdownTcpSocket(state->firstFrameSocket);
        }
    }

    // Join all threads for all streams
    for (int i = 0; i < numActiveVideoStreams; i++) {
        VIDEO_STREAM_STATE* state = &videoStreams[i];

        PltJoinThread(&state->udpPingThread);
        PltJoinThread(&state->receiveThread);
        if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
            PltJoinThread(&state->decoderThread);
        }

        if (state->firstFrameSocket != INVALID_SOCKET) {
            closeSocket(state->firstFrameSocket);
            state->firstFrameSocket = INVALID_SOCKET;
        }
        if (state->rtpSocket != INVALID_SOCKET) {
            closeSocket(state->rtpSocket);
            state->rtpSocket = INVALID_SOCKET;
        }
    }

    VideoCallbacks.cleanup();
}

// Start the video stream(s)
int startVideoStream(void* rendererContext, int drFlags) {
    int err;

    // This must be called before the decoder thread starts submitting
    // decode units
    LC_ASSERT(NegotiatedVideoFormat != 0);
    err = VideoCallbacks.setup(NegotiatedVideoFormat, StreamConfig.width,
        StreamConfig.height, StreamConfig.fps, rendererContext, drFlags);
    if (err != 0) {
        return err;
    }

    // Bind sockets and start threads for each video stream
    for (int i = 0; i < numActiveVideoStreams; i++) {
        VIDEO_STREAM_STATE* state = &videoStreams[i];

        state->firstFrameSocket = INVALID_SOCKET;

        state->rtpSocket = bindUdpSocket(RemoteAddr.ss_family, &LocalAddr, AddrLen,
                                         RTP_RECV_PACKETS_BUFFERED * (StreamConfig.packetSize + MAX_RTP_HEADER_SIZE),
                                         SOCK_QOS_TYPE_VIDEO);
        if (state->rtpSocket == INVALID_SOCKET) {
            // Clean up previously started streams
            for (int j = 0; j < i; j++) {
                closeSocket(videoStreams[j].rtpSocket);
                videoStreams[j].rtpSocket = INVALID_SOCKET;
            }
            VideoCallbacks.cleanup();
            return LastSocketError();
        }
    }

    VideoCallbacks.start();

    // Start receive threads for all streams
    for (int i = 0; i < numActiveVideoStreams; i++) {
        VIDEO_STREAM_STATE* state = &videoStreams[i];
        char threadName[32];

        snprintf(threadName, sizeof(threadName), "VideoRecv%d", i);
        err = PltCreateThread(threadName, VideoReceiveThreadProc, state, &state->receiveThread);
        if (err != 0) {
            // Interrupt and join previously started receive threads
            for (int j = 0; j < i; j++) {
                PltInterruptThread(&videoStreams[j].receiveThread);
                PltJoinThread(&videoStreams[j].receiveThread);
            }
            VideoCallbacks.stop();
            for (int j = 0; j < numActiveVideoStreams; j++) {
                closeSocket(videoStreams[j].rtpSocket);
                videoStreams[j].rtpSocket = INVALID_SOCKET;
            }
            VideoCallbacks.cleanup();
            return err;
        }
    }

    // Start decoder threads for non-direct-submit decoders (stream 0 only for now)
    if ((VideoCallbacks.capabilities & (CAPABILITY_DIRECT_SUBMIT | CAPABILITY_PULL_RENDERER)) == 0) {
        VIDEO_STREAM_STATE* state = &videoStreams[0];
        err = PltCreateThread("VideoDec", VideoDecoderThreadProc, state, &state->decoderThread);
        if (err != 0) {
            VideoCallbacks.stop();
            for (int j = 0; j < numActiveVideoStreams; j++) {
                PltInterruptThread(&videoStreams[j].receiveThread);
                PltJoinThread(&videoStreams[j].receiveThread);
                closeSocket(videoStreams[j].rtpSocket);
                videoStreams[j].rtpSocket = INVALID_SOCKET;
            }
            VideoCallbacks.cleanup();
            return err;
        }
    }

    // Gen 3 first frame handling (stream 0 only)
    if (AppVersionQuad[0] == 3) {
        VIDEO_STREAM_STATE* state = &videoStreams[0];
        state->firstFrameSocket = connectTcpSocket(&RemoteAddr, AddrLen,
                                                   FIRST_FRAME_PORT, FIRST_FRAME_TIMEOUT_SEC);
        if (state->firstFrameSocket == INVALID_SOCKET) {
            stopVideoStream();
            return LastSocketError();
        }
    }

    // Start ping threads for all streams
    for (int i = 0; i < numActiveVideoStreams; i++) {
        VIDEO_STREAM_STATE* state = &videoStreams[i];
        char threadName[32];

        snprintf(threadName, sizeof(threadName), "VideoPing%d", i);
        err = PltCreateThread(threadName, VideoPingThreadProc, state, &state->udpPingThread);
        if (err != 0) {
            // Interrupt and join previously started ping threads
            for (int j = 0; j < i; j++) {
                PltInterruptThread(&videoStreams[j].udpPingThread);
                PltJoinThread(&videoStreams[j].udpPingThread);
            }
            stopVideoStream();
            return err;
        }
    }

    // Gen 3 first frame read (stream 0 only)
    if (AppVersionQuad[0] == 3) {
        err = readFirstFrame(&videoStreams[0]);
        if (err != 0) {
            stopVideoStream();
            return err;
        }
    }

    return 0;
}

const RTP_VIDEO_STATS* LiGetRTPVideoStats(void) {
    return &videoStreams[0].rtpQueue.stats;
}
