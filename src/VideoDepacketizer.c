#include "Limelight-internal.h"

// Uncomment to test 3 byte Annex B start sequences with GFE
//#define FORCE_3_BYTE_START_SEQUENCES

#define DR_CLEANUP -1000

#define CONSECUTIVE_DROP_LIMIT 120

// Per-stream video depacketizer context
typedef struct _VIDEO_DEPACKETIZER_CTX {
    PLENTRY nalChainHead;
    PLENTRY nalChainTail;
    int nalChainDataLength;

    unsigned int nextFrameNumber;
    unsigned int startFrameNumber;
    bool waitingForNextSuccessfulFrame;
    bool waitingForIdrFrame;
    bool waitingForRefInvalFrame;
    unsigned int lastPacketInStream;
    bool decodingFrame;
    int frameType;
    uint16_t lastPacketPayloadLength;
    bool strictIdrFrameWait;
    uint64_t syntheticPtsBaseUs;
    uint16_t frameHostProcessingLatency;
    uint64_t firstPacketReceiveTimeUs;
    uint64_t firstPacketPresentationTime;
    uint32_t firstPacketRtpTimestamp;
    bool dropStatePending;
    bool idrFrameProcessed;

    unsigned int consecutiveFrameDrops;

    LINKED_BLOCKING_QUEUE decodeUnitQueue;

    int streamIndex;
} VIDEO_DEPACKETIZER_CTX;

static VIDEO_DEPACKETIZER_CTX depacketizers[MAX_VIDEO_STREAMS];

// Validates a stream index before it is used to index depacketizers[]. An out-of-range
// value means memory corruption or a malformed/hostile value reached us, so we assert in
// debug builds and fail safe in release builds rather than reading or writing out of bounds.
static bool isValidStreamIndex(int streamIndex) {
    if (streamIndex < 0 || streamIndex >= MAX_VIDEO_STREAMS) {
        LC_ASSERT(false);
        return false;
    }
    return true;
}

typedef struct _BUFFER_DESC {
    char* data;
    unsigned int offset;
    unsigned int length;
} BUFFER_DESC, *PBUFFER_DESC;

typedef struct _LENTRY_INTERNAL {
    LENTRY entry;
    void* allocPtr;
} LENTRY_INTERNAL, *PLENTRY_INTERNAL;

#define H264_NAL_TYPE(x) ((x) & 0x1F)
#define HEVC_NAL_TYPE(x) (((x) & 0x7E) >> 1)

#define H264_NAL_TYPE_SEI 6
#define H264_NAL_TYPE_SPS 7
#define H264_NAL_TYPE_PPS 8
#define H264_NAL_TYPE_AUD 9
#define H264_NAL_TYPE_FILLER 12
#define HEVC_NAL_TYPE_VPS 32
#define HEVC_NAL_TYPE_SPS 33
#define HEVC_NAL_TYPE_PPS 34
#define HEVC_NAL_TYPE_AUD 35
#define HEVC_NAL_TYPE_FILLER 38
#define HEVC_NAL_TYPE_SEI 39

// Init
void initializeVideoDepacketizer(int streamIndex, int pktSize) {
    VIDEO_DEPACKETIZER_CTX* ctx;

    if (!isValidStreamIndex(streamIndex)) {
        return;
    }
    ctx = &depacketizers[streamIndex];

    // Each stream owns its own decode unit queue, so its frames are delivered only
    // to the decoder bound to that stream's monitor. initializeVideoDepacketizer()
    // and destroyVideoDepacketizer() are both called exactly once per stream, so the
    // queue's mutex/condvar are created and destroyed in balanced pairs.
    LbqInitializeLinkedBlockingQueue(&ctx->decodeUnitQueue, 15);

    ctx->nextFrameNumber = 1;
    ctx->startFrameNumber = 0;
    ctx->waitingForNextSuccessfulFrame = false;
    ctx->waitingForIdrFrame = true;
    ctx->waitingForRefInvalFrame = false;
    ctx->lastPacketInStream = UINT32_MAX;
    ctx->decodingFrame = false;
    ctx->syntheticPtsBaseUs = 0;
    ctx->frameHostProcessingLatency = 0;
    ctx->firstPacketReceiveTimeUs = 0;
    ctx->firstPacketPresentationTime = 0;
    ctx->firstPacketRtpTimestamp = 0;
    ctx->lastPacketPayloadLength = 0;
    ctx->dropStatePending = false;
    ctx->idrFrameProcessed = false;
    ctx->consecutiveFrameDrops = 0;
    ctx->nalChainHead = NULL;
    ctx->nalChainTail = NULL;
    ctx->nalChainDataLength = 0;
    ctx->strictIdrFrameWait = !isReferenceFrameInvalidationEnabled();
    ctx->streamIndex = streamIndex;
}

// Free the NAL chain
static void cleanupFrameState(VIDEO_DEPACKETIZER_CTX* ctx) {
    PLENTRY_INTERNAL lastEntry;

    while (ctx->nalChainHead != NULL) {
        lastEntry = (PLENTRY_INTERNAL)ctx->nalChainHead;
        ctx->nalChainHead = lastEntry->entry.next;
        free(lastEntry->allocPtr);
    }

    ctx->nalChainTail = NULL;

    ctx->nalChainDataLength = 0;
}

// Cleanup frame state and set that we're waiting for an IDR Frame
static void dropFrameState(VIDEO_DEPACKETIZER_CTX* ctx) {
    // This may only be called at frame boundaries
    LC_ASSERT(!ctx->decodingFrame);

    // We're dropping frame state now
    ctx->dropStatePending = false;

    if (ctx->strictIdrFrameWait || !ctx->idrFrameProcessed || ctx->waitingForIdrFrame) {
        // We'll need an IDR frame now if we're in non-RFI mode, if we've never
        // received an IDR frame, or if we explicitly need an IDR frame.
        ctx->waitingForIdrFrame = true;
    }
    else {
        ctx->waitingForRefInvalFrame = true;
    }

    // Count the number of consecutive frames dropped
    ctx->consecutiveFrameDrops++;

    // If we reach our limit, immediately request an IDR frame and reset
    if (ctx->consecutiveFrameDrops == CONSECUTIVE_DROP_LIMIT) {
        Limelog("Reached consecutive drop limit\n");

        // Restart the count
        ctx->consecutiveFrameDrops = 0;

        // Request an IDR frame
        ctx->waitingForIdrFrame = true;
        LiRequestIdrFrame();
    }

    cleanupFrameState(ctx);
}

// Cleanup the list of decode units
static void freeDecodeUnitList(PLINKED_BLOCKING_QUEUE_ENTRY entry) {
    PLINKED_BLOCKING_QUEUE_ENTRY nextEntry;

    while (entry != NULL) {
        nextEntry = entry->flink;

        // Complete this with a failure status
        LiCompleteVideoFrame(entry->data, DR_CLEANUP);

        entry = nextEntry;
    }
}

void stopVideoDepacketizer(int streamIndex) {
    if (!isValidStreamIndex(streamIndex)) {
        return;
    }
    // Wake any consumer blocked in LiWaitForNextVideoFrameForStream() on this stream.
    LbqSignalQueueShutdown(&depacketizers[streamIndex].decodeUnitQueue);
}

// Cleanup video depacketizer and free malloced memory
void destroyVideoDepacketizer(int streamIndex) {
    VIDEO_DEPACKETIZER_CTX* ctx;

    if (!isValidStreamIndex(streamIndex)) {
        return;
    }
    ctx = &depacketizers[streamIndex];

    // Destroy this stream's own decode unit queue
    freeDecodeUnitList(LbqDestroyLinkedBlockingQueue(&ctx->decodeUnitQueue));

    cleanupFrameState(ctx);
}

// NB: This function also ensures an additional byte for the NALU type exists after the start sequence
static bool getAnnexBStartSequence(PBUFFER_DESC current, PBUFFER_DESC startSeq) {
    // We must not get called for other codecs
    LC_ASSERT(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265));

    if (current->length <= 3) {
        return false;
    }

    if (current->data[current->offset] == 0 &&
        current->data[current->offset + 1] == 0) {
        if (current->data[current->offset + 2] == 0) {
            if (current->length > 4 && current->data[current->offset + 3] == 1) {
                // Frame start
                if (startSeq != NULL) {
                    startSeq->data = current->data;
                    startSeq->offset = current->offset;
                    startSeq->length = 4;
                }
                return true;
            }
        }
        else if (current->data[current->offset + 2] == 1) {
            // NAL start
            if (startSeq != NULL) {
                startSeq->data = current->data;
                startSeq->offset = current->offset;
                startSeq->length = 3;
            }
            return true;
        }
    }

    return false;
}

static void validateDecodeUnitForPlayback(VIDEO_DEPACKETIZER_CTX* ctx, PDECODE_UNIT decodeUnit) {
    // Frames must always have at least one buffer
    LC_ASSERT(decodeUnit->bufferList != NULL);
    LC_ASSERT(decodeUnit->fullLength != 0);

    // Validate the buffers in the frame
    if (decodeUnit->frameType == FRAME_TYPE_IDR) {
        // IDR frames always start with codec configuration data
        if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
            // H.264 IDR frames should have an SPS, PPS, then picture data
            LC_ASSERT_VT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_SPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next != NULL);
            LC_ASSERT_VT(decodeUnit->bufferList->next->bufferType == BUFFER_TYPE_PPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next != NULL);
        }
        else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
            // HEVC IDR frames should have an VPS, SPS, PPS, then picture data
            LC_ASSERT_VT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_VPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next != NULL);
            LC_ASSERT_VT(decodeUnit->bufferList->next->bufferType == BUFFER_TYPE_SPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next != NULL);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next->bufferType == BUFFER_TYPE_PPS);
            LC_ASSERT_VT(decodeUnit->bufferList->next->next->next != NULL);
        }
        else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_AV1) {
            // We don't parse the AV1 bitstream
            LC_ASSERT_VT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_PICDATA);
        }
        else {
            LC_ASSERT(false);
        }
    }
    else {
        LC_ASSERT(decodeUnit->frameType == FRAME_TYPE_PFRAME);

        // P frames always start with picture data
        LC_ASSERT(decodeUnit->bufferList->bufferType == BUFFER_TYPE_PICDATA);

        // We must not dequeue a P frame before an IDR frame has been successfully processed
        LC_ASSERT(ctx->idrFrameProcessed);
    }
}

bool LiWaitForNextVideoFrameForStream(int streamIndex, VIDEO_FRAME_HANDLE* frameHandle, PDECODE_UNIT* decodeUnit) {
    PQUEUED_DECODE_UNIT qdu;

    if (!isValidStreamIndex(streamIndex)) {
        return false;
    }

    int err = LbqWaitForQueueElement(&depacketizers[streamIndex].decodeUnitQueue, (void**)&qdu);
    if (err != LBQ_SUCCESS) {
        return false;
    }

    if (isValidStreamIndex(qdu->decodeUnit.streamIndex)) {
        validateDecodeUnitForPlayback(&depacketizers[qdu->decodeUnit.streamIndex], &qdu->decodeUnit);
    }

    *frameHandle = qdu;
    *decodeUnit = &qdu->decodeUnit;
    return true;
}

bool LiWaitForNextVideoFrame(VIDEO_FRAME_HANDLE* frameHandle, PDECODE_UNIT* decodeUnit) {
    return LiWaitForNextVideoFrameForStream(0, frameHandle, decodeUnit);
}

bool LiPollNextVideoFrameForStream(int streamIndex, VIDEO_FRAME_HANDLE* frameHandle, PDECODE_UNIT* decodeUnit) {
    PQUEUED_DECODE_UNIT qdu;

    if (!isValidStreamIndex(streamIndex)) {
        return false;
    }

    int err = LbqPollQueueElement(&depacketizers[streamIndex].decodeUnitQueue, (void**)&qdu);
    if (err != LBQ_SUCCESS) {
        return false;
    }

    if (isValidStreamIndex(qdu->decodeUnit.streamIndex)) {
        validateDecodeUnitForPlayback(&depacketizers[qdu->decodeUnit.streamIndex], &qdu->decodeUnit);
    }

    *frameHandle = qdu;
    *decodeUnit = &qdu->decodeUnit;
    return true;
}

bool LiPollNextVideoFrame(VIDEO_FRAME_HANDLE* frameHandle, PDECODE_UNIT* decodeUnit) {
    return LiPollNextVideoFrameForStream(0, frameHandle, decodeUnit);
}

bool LiPeekNextVideoFrameForStream(int streamIndex, PDECODE_UNIT* decodeUnit) {
    PQUEUED_DECODE_UNIT qdu;

    if (!isValidStreamIndex(streamIndex)) {
        return false;
    }

    int err = LbqPeekQueueElement(&depacketizers[streamIndex].decodeUnitQueue, (void**)&qdu);
    if (err != LBQ_SUCCESS) {
        return false;
    }

    if (isValidStreamIndex(qdu->decodeUnit.streamIndex)) {
        validateDecodeUnitForPlayback(&depacketizers[qdu->decodeUnit.streamIndex], &qdu->decodeUnit);
    }

    *decodeUnit = &qdu->decodeUnit;
    return true;
}

bool LiPeekNextVideoFrame(PDECODE_UNIT* decodeUnit) {
    return LiPeekNextVideoFrameForStream(0, decodeUnit);
}

void LiWakeWaitForVideoFrameForStream(int streamIndex) {
    if (!isValidStreamIndex(streamIndex)) {
        return;
    }
    LbqSignalQueueUserWake(&depacketizers[streamIndex].decodeUnitQueue);
}

void LiWakeWaitForVideoFrame(void) {
    LiWakeWaitForVideoFrameForStream(0);
}

// Cleanup a decode unit by freeing the buffer chain and the holder
void LiCompleteVideoFrame(VIDEO_FRAME_HANDLE handle, int drStatus) {
    PQUEUED_DECODE_UNIT qdu = handle;
    PLENTRY_INTERNAL lastEntry;
    VIDEO_DEPACKETIZER_CTX* ctx = isValidStreamIndex(qdu->decodeUnit.streamIndex) ?
        &depacketizers[qdu->decodeUnit.streamIndex] : NULL;

    if (ctx != NULL && drStatus == DR_NEED_IDR) {
        Limelog("Requesting IDR frame on behalf of DR\n");
        requestDecoderRefresh(ctx->streamIndex);
    }
    else if (ctx != NULL && drStatus == DR_OK && qdu->decodeUnit.frameType == FRAME_TYPE_IDR) {
        // Remember that the IDR frame was processed. We can now use
        // reference frame invalidation.
        ctx->idrFrameProcessed = true;
    }

    while (qdu->decodeUnit.bufferList != NULL) {
        lastEntry = (PLENTRY_INTERNAL)qdu->decodeUnit.bufferList;
        qdu->decodeUnit.bufferList = lastEntry->entry.next;
        free(lastEntry->allocPtr);
    }

    // We will have stack-allocated entries iff we have a direct-submit decoder
    if ((VideoCallbacks.capabilities & CAPABILITY_DIRECT_SUBMIT) == 0) {
        free(qdu);
    }
}

static bool isSeqReferenceFrameStart(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == 5;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        switch (HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length])) {
            case 16:
            case 17:
            case 18:
            case 19:
            case 20:
            case 21:
                return true;

            default:
                return false;
        }
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

static bool isAccessUnitDelimiter(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_AUD;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_AUD;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

static bool isSeiNal(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_SEI;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_SEI;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

#ifdef LC_DEBUG
static bool isFillerDataNal(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_FILLER;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_FILLER;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}
#endif

static bool isPictureParameterSetNal(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_PPS;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_PPS;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

// Advance the buffer descriptor to the start of the next NAL or end of buffer
static void skipToNextNalOrEnd(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    // If we're starting on a NAL boundary, skip to the next one
    if (getAnnexBStartSequence(buffer, &startSeq)) {
        buffer->offset += startSeq.length;
        buffer->length -= startSeq.length;
    }

    // Loop until we find an Annex B start sequence (3 or 4 byte)
    while (!getAnnexBStartSequence(buffer, NULL)) {
        if (buffer->length == 0) {
            // Reached the end of the buffer
            return;
        }

        buffer->offset++;
        buffer->length--;
    }
}

// Advance the buffer descriptor to the start of the next NAL
static void skipToNextNal(PBUFFER_DESC buffer) {
    skipToNextNalOrEnd(buffer);

    // If we skipped all the data, something has gone horribly wrong
    LC_ASSERT(buffer->length > 0);
}

static bool isIdrFrameStart(PBUFFER_DESC buffer) {
    BUFFER_DESC startSeq;

    if (!getAnnexBStartSequence(buffer, &startSeq)) {
        return false;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        return H264_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == H264_NAL_TYPE_SPS;
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        return HEVC_NAL_TYPE(startSeq.data[startSeq.offset + startSeq.length]) == HEVC_NAL_TYPE_VPS;
    }
    else {
        LC_ASSERT(false);
        return false;
    }
}

// Reassemble the frame with the given frame number
static void reassembleFrame(VIDEO_DEPACKETIZER_CTX* ctx, int frameNumber, bool frameIsLTR) {
    if (ctx->nalChainHead != NULL) {
        QUEUED_DECODE_UNIT qduDS;
        PQUEUED_DECODE_UNIT qdu;

        // Use a stack allocation if we won't be queuing this
        if ((VideoCallbacks.capabilities & CAPABILITY_DIRECT_SUBMIT) == 0) {
            qdu = (PQUEUED_DECODE_UNIT)malloc(sizeof(*qdu));
        }
        else {
            qdu = &qduDS;
        }

        if (qdu != NULL) {
            qdu->decodeUnit.bufferList = ctx->nalChainHead;
            qdu->decodeUnit.fullLength = ctx->nalChainDataLength;
            qdu->decodeUnit.frameType = ctx->frameType;
            qdu->decodeUnit.frameNumber = frameNumber;
            qdu->decodeUnit.frameHostProcessingLatency = ctx->frameHostProcessingLatency;
            qdu->decodeUnit.receiveTimeUs = ctx->firstPacketReceiveTimeUs;
            qdu->decodeUnit.presentationTimeUs = ctx->firstPacketPresentationTime;
            qdu->decodeUnit.rtpTimestamp = ctx->firstPacketRtpTimestamp;
            qdu->decodeUnit.enqueueTimeUs = PltGetMicroseconds();

            // These might be wrong for a few frames during a transition between SDR and HDR,
            // but the effects shouldn't very noticable since that's an infrequent operation.
            //
            // If we start sending this state in the frame header, we can make it 100% accurate.
            qdu->decodeUnit.hdrActive = LiGetCurrentHostDisplayHdrMode();
            qdu->decodeUnit.colorspace = (uint8_t)(qdu->decodeUnit.hdrActive ? COLORSPACE_REC_2020 : StreamConfig.colorSpace);
            qdu->decodeUnit.streamIndex = ctx->streamIndex;

            // Invoke the key frame callback if needed
            if (ctx->nalChainHead->bufferType != BUFFER_TYPE_PICDATA || qdu->decodeUnit.frameType == FRAME_TYPE_IDR) {
                qdu->decodeUnit.frameType = FRAME_TYPE_IDR;
                notifyKeyFrameReceived(ctx->streamIndex);
            }
            else {
                qdu->decodeUnit.frameType = FRAME_TYPE_PFRAME;
            }

            ctx->nalChainHead = ctx->nalChainTail = NULL;
            ctx->nalChainDataLength = 0;

            if ((VideoCallbacks.capabilities & CAPABILITY_DIRECT_SUBMIT) == 0) {
                // Each stream pushes to its own decode unit queue so the frame reaches
                // the decoder bound to that stream's monitor.
                if (LbqOfferQueueItem(&depacketizers[ctx->streamIndex].decodeUnitQueue, qdu, &qdu->entry) == LBQ_BOUND_EXCEEDED) {
                    Limelog("Video decode unit queue overflow (stream %d)\n", ctx->streamIndex);

                    // RFI recovery is not supported here
                    ctx->waitingForIdrFrame = true;

                    // Clear NAL state for the frame that we failed to enqueue
                    ctx->nalChainHead = qdu->decodeUnit.bufferList;
                    ctx->nalChainDataLength = qdu->decodeUnit.fullLength;
                    dropFrameState(ctx);

                    // Free the DU we were going to queue
                    free(qdu);

                    // Free all frames in this stream's decode unit queue
                    freeDecodeUnitList(LbqFlushQueueItems(&depacketizers[ctx->streamIndex].decodeUnitQueue));

                    // Request an IDR frame to recover this stream
                    LiRequestIdrFrameForStream((uint8_t) ctx->streamIndex);
                    return;
                }
            }
            else {
                // Submit the frame to the decoder
                validateDecodeUnitForPlayback(ctx, &qdu->decodeUnit);
                LiCompleteVideoFrame(qdu, VideoCallbacks.submitDecodeUnit(&qdu->decodeUnit));
            }

            // Notify the control connection
            connectionReceivedCompleteFrame(ctx->streamIndex, frameNumber, frameIsLTR);

            // Clear frame drops
            ctx->consecutiveFrameDrops = 0;

            // Move the start of our (potential) RFI window to the next frame
            ctx->startFrameNumber = ctx->nextFrameNumber;
        }
    }
}

static int getBufferFlags(char* data, int length) {
    BUFFER_DESC buffer;
    BUFFER_DESC candidate;

    // We only parse H.264 and HEVC bitstreams
    if (!(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265))) {
        return BUFFER_TYPE_PICDATA;
    }

    buffer.data = data;
    buffer.length = (unsigned int)length;
    buffer.offset = 0;

    if (!getAnnexBStartSequence(&buffer, &candidate)) {
        return BUFFER_TYPE_PICDATA;
    }

    if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H264) {
        switch (H264_NAL_TYPE(candidate.data[candidate.offset + candidate.length])) {
        case H264_NAL_TYPE_SPS:
            return BUFFER_TYPE_SPS;

        case H264_NAL_TYPE_PPS:
            return BUFFER_TYPE_PPS;

        default:
            return BUFFER_TYPE_PICDATA;
        }
    }
    else if (NegotiatedVideoFormat & VIDEO_FORMAT_MASK_H265) {
        switch (HEVC_NAL_TYPE(candidate.data[candidate.offset + candidate.length])) {
            case HEVC_NAL_TYPE_SPS:
                return BUFFER_TYPE_SPS;

            case HEVC_NAL_TYPE_PPS:
                return BUFFER_TYPE_PPS;

            case HEVC_NAL_TYPE_VPS:
                return BUFFER_TYPE_VPS;

            default:
                return BUFFER_TYPE_PICDATA;
        }
    }
    else {
        LC_ASSERT(false);
        return BUFFER_TYPE_PICDATA;
    }
}

// As an optimization, we can cast the existing packet buffer to a PLENTRY and avoid
// a malloc() and a memcpy() of the packet data.
static void queueFragment(VIDEO_DEPACKETIZER_CTX* ctx, PLENTRY_INTERNAL* existingEntry, char* data, int offset, int length) {
    PLENTRY_INTERNAL entry;

    if (existingEntry == NULL || *existingEntry == NULL) {
        entry = (PLENTRY_INTERNAL)malloc(sizeof(*entry) + length);
    }
    else {
        entry = *existingEntry;
    }

    if (entry != NULL) {
        entry->entry.next = NULL;
        entry->entry.length = length;

        // If we had to allocate a new entry, we must copy the data. If not,
        // the data already resides within the LENTRY allocation.
        if (existingEntry == NULL || *existingEntry == NULL) {
            entry->allocPtr = entry;

            entry->entry.data = (char*)(entry + 1);
            memcpy(entry->entry.data, &data[offset], entry->entry.length);
        }
        else {
            entry->entry.data = &data[offset];

            // The caller should have already set this up for us
            LC_ASSERT(entry->allocPtr != NULL);

            // We now own the packet buffer and will manage freeing it
            *existingEntry = NULL;
        }

        entry->entry.bufferType = getBufferFlags(entry->entry.data, entry->entry.length);

        ctx->nalChainDataLength += entry->entry.length;

        if (ctx->nalChainTail == NULL) {
            LC_ASSERT(ctx->nalChainHead == NULL);
            ctx->nalChainHead = ctx->nalChainTail = (PLENTRY)entry;
        }
        else {
            LC_ASSERT(ctx->nalChainHead != NULL);
            ctx->nalChainTail->next = (PLENTRY)entry;
            ctx->nalChainTail = ctx->nalChainTail->next;
        }
    }
}

// Process an RTP Payload using the slow path that handles multiple NALUs per packet
static void processAvcHevcRtpPayloadSlow(VIDEO_DEPACKETIZER_CTX* ctx, PBUFFER_DESC currentPos, PLENTRY_INTERNAL* existingEntry) {
    // We should not have any NALUs when processing the first packet in an IDR frame
    LC_ASSERT(ctx->nalChainHead == NULL);
    LC_ASSERT(ctx->nalChainTail == NULL);

    while (currentPos->length != 0) {
        // Skip through any padding bytes
        if (!getAnnexBStartSequence(currentPos, NULL)) {
            skipToNextNal(currentPos);
        }

        // Skip any prepended AUD or SEI NALUs. We may have padding between
        // these on IDR frames, so the check in processRtpPayload() is not
        // completely sufficient to handle that case.
        while (isAccessUnitDelimiter(currentPos) || isSeiNal(currentPos)) {
            skipToNextNal(currentPos);
        }

        int start = currentPos->offset;
        bool containsPicData = false;

#ifdef FORCE_3_BYTE_START_SEQUENCES
        start++;
#endif

        if (isSeqReferenceFrameStart(currentPos)) {
            // No longer waiting for an IDR frame
            ctx->waitingForIdrFrame = false;
            ctx->waitingForRefInvalFrame = false;

            // Cancel any pending IDR frame request
            ctx->waitingForNextSuccessfulFrame = false;

            // Use the cached LENTRY for this NALU since it will be
            // the bulk of the data in this packet.
            containsPicData = true;

            // This is an IDR frame
            ctx->frameType = FRAME_TYPE_IDR;
        }

        // Move to the next NALU
        skipToNextNalOrEnd(currentPos);

        // If this is the picture data, we expect it to extend to the end of the packet
        if (containsPicData) {
            while (currentPos->length != 0) {
                // Any NALUs we encounter on the way to the end of the packet must be
                // reference frame slices or filler data.
                LC_ASSERT_VT(isSeqReferenceFrameStart(currentPos) || isFillerDataNal(currentPos));
                skipToNextNalOrEnd(currentPos);
            }
        }

        // To minimize copies, we'll allocate for SPS, PPS, and VPS to allow
        // us to reuse the packet buffer for the picture data in the I-frame.
        queueFragment(ctx, containsPicData ? existingEntry : NULL,
                      currentPos->data, start, currentPos->offset - start);
    }
}

// Dumps the decode unit queue and ensures the next frame submitted to the decoder will be
// an IDR frame
void requestDecoderRefresh(int streamIndex) {
    VIDEO_DEPACKETIZER_CTX* ctx;

    if (!isValidStreamIndex(streamIndex)) {
        return;
    }
    ctx = &depacketizers[streamIndex];

    // Wait for the next IDR frame
    ctx->waitingForIdrFrame = true;

    // Flush this stream's decode unit queue
    freeDecodeUnitList(LbqFlushQueueItems(&ctx->decodeUnitQueue));

    // Request the receive thread drop its state
    // on the next call. We can't do it here because
    // it may be trying to queue DUs and we'll nuke
    // the state out from under it.
    ctx->dropStatePending = true;

    // Request the IDR frame
    LiRequestIdrFrame();
}

// Return 1 if packet is the first one in the frame
static bool isFirstPacket(uint8_t flags, uint8_t fecBlockNumber) {
    // Clear the picture data flag
    flags &= ~FLAG_CONTAINS_PIC_DATA;

    // Check if it's just the start or both start and end of a frame
    return (flags == (FLAG_SOF | FLAG_EOF) || flags == FLAG_SOF) && fecBlockNumber == 0;
}

// Process an RTP Payload
// The caller will free *existingEntry unless we NULL it
static void processRtpPayload(VIDEO_DEPACKETIZER_CTX* ctx, PNV_VIDEO_PACKET videoPacket, int length,
                       uint64_t receiveTimeUs, uint64_t presentationTimeUs, uint32_t rtpTimestamp,
                       PLENTRY_INTERNAL* existingEntry) {
    BUFFER_DESC currentPos;
    uint32_t frameIndex;
    uint8_t flags;
    uint8_t extraFlags;
    bool firstPacket, lastPacket;
    uint32_t streamPacketIndex;
    uint8_t fecCurrentBlockNumber;
    uint8_t fecLastBlockNumber;

    // Mask the top 8 bits from the SPI
    videoPacket->streamPacketIndex >>= 8;
    videoPacket->streamPacketIndex &= 0xFFFFFF;

    currentPos.data = (char*)(videoPacket + 1);
    currentPos.offset = 0;
    currentPos.length = length - sizeof(*videoPacket);

    fecCurrentBlockNumber = (videoPacket->multiFecBlocks >> 4) & 0x3;
    fecLastBlockNumber = (videoPacket->multiFecBlocks >> 6) & 0x3;
    frameIndex = videoPacket->frameIndex;
    flags = videoPacket->flags;
    extraFlags = videoPacket->extraFlags;
    firstPacket = isFirstPacket(flags, fecCurrentBlockNumber);
    lastPacket = (flags & FLAG_EOF) && fecCurrentBlockNumber == fecLastBlockNumber;

    LC_ASSERT_VT((flags & ~(FLAG_SOF | FLAG_EOF | FLAG_CONTAINS_PIC_DATA)) == 0);

    streamPacketIndex = videoPacket->streamPacketIndex;

    // Drop packets from a previously corrupt frame
    if (isBefore32(frameIndex, ctx->nextFrameNumber)) {
        return;
    }

    // The FEC queue can sometimes recover corrupt frames (see comments in RtpFecQueue).
    // It almost always detects them before they get to us, but in case it doesn't
    // the streamPacketIndex not matching correctly should find nearly all of the rest.
    if (isBefore24(streamPacketIndex, U24(ctx->lastPacketInStream + 1)) ||
            (!(flags & FLAG_SOF) && streamPacketIndex != U24(ctx->lastPacketInStream + 1))) {
        Limelog("Depacketizer detected corrupt frame: %d", frameIndex);
        ctx->decodingFrame = false;
        ctx->nextFrameNumber = frameIndex + 1;
        dropFrameState(ctx);
        if (ctx->waitingForIdrFrame) {
            LiRequestIdrFrame();
        }
        else {
            connectionDetectedFrameLoss(ctx->startFrameNumber, frameIndex);
        }
        return;
    }

    // Verify that we didn't receive an incomplete frame
    LC_ASSERT(firstPacket ^ ctx->decodingFrame);

    // Check sequencing of this frame to ensure we didn't
    // miss one in between
    if (firstPacket) {
        // Make sure this is the next consecutive frame
        if (isBefore32(ctx->nextFrameNumber, frameIndex)) {
            if (ctx->nextFrameNumber + 1 == frameIndex) {
                Limelog("Network dropped 1 frame (frame %d)\n", frameIndex - 1);
            }
            else {
                Limelog("Network dropped %d frames (frames %d to %d)\n",
                        frameIndex - ctx->nextFrameNumber,
                        ctx->nextFrameNumber,
                        frameIndex - 1);
            }

            ctx->nextFrameNumber = frameIndex;

            // Wait until next complete frame
            ctx->waitingForNextSuccessfulFrame = true;
            dropFrameState(ctx);
        }
        else {
            LC_ASSERT(ctx->nextFrameNumber == frameIndex);
        }

        // We're now decoding a frame
        ctx->decodingFrame = true;
        ctx->frameType = FRAME_TYPE_PFRAME;
        ctx->firstPacketReceiveTimeUs = receiveTimeUs;

        // Some versions of Sunshine don't send a valid PTS, so we will
        // synthesize one using the receive time as the time base.
        if (!ctx->syntheticPtsBaseUs) {
            ctx->syntheticPtsBaseUs = receiveTimeUs;
        }

        if (!presentationTimeUs && frameIndex > 0) {
            ctx->firstPacketPresentationTime = receiveTimeUs - ctx->syntheticPtsBaseUs;
        }
        else {
            ctx->firstPacketPresentationTime = presentationTimeUs;
        }

        ctx->firstPacketRtpTimestamp = rtpTimestamp;
    }

    ctx->lastPacketInStream = streamPacketIndex;

    // If this is the first packet, skip the frame header (if one exists)
    uint32_t frameHeaderSize;
    LC_ASSERT_VT(currentPos.length > 0);
    if (firstPacket && currentPos.length > 0) {
        // Parse the frame type from the header
        LC_ASSERT_VT(currentPos.length >= 4);
        if (APP_VERSION_AT_LEAST(7, 1, 350) && currentPos.length >= 4) {
            switch (currentPos.data[currentPos.offset + 3]) {
            case 1: // Normal P-frame
                break;
            case 2: // IDR frame
                // For other codecs, we trust the frame header rather than parsing the bitstream
                // to determine if a given frame is an IDR frame.
                if (!(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265))) {
                    ctx->waitingForIdrFrame = false;
                    ctx->waitingForNextSuccessfulFrame = false;
                    ctx->frameType = FRAME_TYPE_IDR;
                }
                // Fall-through
            case 4: // Intra-refresh
            case 5: // P-frame with reference frames invalidated
                if (ctx->waitingForRefInvalFrame) {
                    Limelog("Next post-invalidation frame is: %d (%s-frame)\n",
                            frameIndex,
                            currentPos.data[currentPos.offset + 3] == 5 ? "P" : "I");
                    ctx->waitingForRefInvalFrame = false;
                    ctx->waitingForNextSuccessfulFrame = false;
                }
                break;
            case 104: // Sunshine hardcoded header
                break;
            default:
                Limelog("Unrecognized frame type: %d", currentPos.data[currentPos.offset + 3]);
                LC_ASSERT_VT(false);
                break;
            }
        }
        else {
            // Hope for the best with older servers
            if (ctx->waitingForRefInvalFrame) {
                connectionDetectedFrameLoss(ctx->startFrameNumber, frameIndex - 1);
                ctx->waitingForRefInvalFrame = false;
                ctx->waitingForNextSuccessfulFrame = false;
            }
        }

        // Sunshine can provide host processing latency of the frame
        LC_ASSERT_VT(currentPos.length >= 3);
        if (IS_SUNSHINE() && currentPos.length >= 3) {
            BYTE_BUFFER bb;
            BbInitializeWrappedBuffer(&bb, currentPos.data, currentPos.offset + 1, 2, BYTE_ORDER_LITTLE);
            BbGet16(&bb, &ctx->frameHostProcessingLatency);
        }

        // Codecs like H.264 and HEVC handle the FEC trailing zero padding just fine, but other
        // codecs need the exact length encoded separately.
        LC_ASSERT_VT(currentPos.length >= 6);
        if (!(NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265)) && currentPos.length >= 6) {
            BYTE_BUFFER bb;
            BbInitializeWrappedBuffer(&bb, currentPos.data, currentPos.offset + 4, 2, BYTE_ORDER_LITTLE);
            BbGet16(&bb, &ctx->lastPacketPayloadLength);
        }

        if (APP_VERSION_AT_LEAST(7, 1, 450)) {
            // >= 7.1.450 uses 2 different header lengths based on the first byte:
            // 0x01 indicates an 8 byte header
            // 0x81 indicates a 44 byte header
            if (currentPos.data[0] == 0x01) {
                frameHeaderSize = 8;
            }
            else {
                LC_ASSERT_VT(currentPos.data[0] == (char)0x81);
                frameHeaderSize = 44;
            }
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 446)) {
            // [7.1.446, 7.1.450) uses 2 different header lengths based on the first byte:
            // 0x01 indicates an 8 byte header
            // 0x81 indicates a 41 byte header
            if (currentPos.data[0] == 0x01) {
                frameHeaderSize = 8;
            }
            else {
                LC_ASSERT_VT(currentPos.data[0] == (char)0x81);
                frameHeaderSize = 41;
            }
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 415)) {
            // [7.1.415, 7.1.446) uses 2 different header lengths based on the first byte:
            // 0x01 indicates an 8 byte header
            // 0x81 indicates a 24 byte header
            if (currentPos.data[0] == 0x01) {
                frameHeaderSize = 8;
            }
            else {
                LC_ASSERT_VT(currentPos.data[0] == (char)0x81);
                frameHeaderSize = 24;
            }
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 350)) {
            // [7.1.350, 7.1.415) should use the 8 byte header again
            frameHeaderSize = 8;
        }
        else if (APP_VERSION_AT_LEAST(7, 1, 320)) {
            // [7.1.320, 7.1.350) should use the 12 byte frame header
            frameHeaderSize = 12;
        }
        else if (APP_VERSION_AT_LEAST(5, 0, 0)) {
            // [5.x, 7.1.320) should use the 8 byte header
            frameHeaderSize = 8;
        }
        else {
            // Other versions don't have a frame header at all
            frameHeaderSize = 0;
        }

        LC_ASSERT_VT(currentPos.length >= frameHeaderSize);
        if (currentPos.length >= frameHeaderSize) {
            // Skip past the frame header
            currentPos.offset += frameHeaderSize;
            currentPos.length -= frameHeaderSize;
        }

        // We only parse H.264 and HEVC at the NALU level
        if (NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265)) {
            // The Annex B NALU start prefix must be next
            if (!getAnnexBStartSequence(&currentPos, NULL)) {
                // If we aren't starting on a start prefix, something went wrong.
                LC_ASSERT_VT(false);

                // For release builds, we will try to recover by searching for one.
                // This mimics the way most decoders handle this situation.
                skipToNextNal(&currentPos);
            }

            // If an AUD NAL is prepended to this frame data, remove it.
            // Other parts of this code are not prepared to deal with a
            // NAL of that type, so stripping it is the easiest option.
            if (isAccessUnitDelimiter(&currentPos)) {
                skipToNextNal(&currentPos);
            }

            // There may be one or more SEI NAL units prepended to the
            // frame data *after* the (optional) AUD.
            while (isSeiNal(&currentPos)) {
                skipToNextNal(&currentPos);
            }
        }
    }
    else {
        // There is no frame header on later packets
        frameHeaderSize = 0;
    }

    if (NegotiatedVideoFormat & (VIDEO_FORMAT_MASK_H264 | VIDEO_FORMAT_MASK_H265)) {
        if (firstPacket && isIdrFrameStart(&currentPos)) {
            // SPS and PPS prefix is padded between NALs, so we must decode it with the slow path
            processAvcHevcRtpPayloadSlow(ctx, &currentPos, existingEntry);
        }
        else {
            // Intel's H.264 Media Foundation encoder prepends a PPS to each P-frame.
            // Skip it to avoid confusing clients.
            if (firstPacket && isPictureParameterSetNal(&currentPos)) {
                skipToNextNal(&currentPos);
            }

#ifdef FORCE_3_BYTE_START_SEQUENCES
            if (firstPacket) {
                currentPos.offset++;
                currentPos.length--;
            }
#endif

            queueFragment(ctx, existingEntry, currentPos.data, currentPos.offset, currentPos.length);
        }
    }
    else {
        // We fixup the length of the last packet for other codecs since they may not be tolerant
        // of trailing zero padding like H.264/HEVC Annex B bitstream parsers are.
        if (lastPacket) {
            // The payload length includes the frame header, so it cannot be smaller than that
            LC_ASSERT_VT(ctx->lastPacketPayloadLength > frameHeaderSize);

            // The payload length cannot be smaller than the actual received payload
            // NB: currentPos.length is already adjusted to exclude the frameHeaderSize from above
            LC_ASSERT_VT(ctx->lastPacketPayloadLength - frameHeaderSize <= currentPos.length);

            // If the payload length is valid, truncate the packet. If not, discard this frame.
            if (ctx->lastPacketPayloadLength > frameHeaderSize && ctx->lastPacketPayloadLength - frameHeaderSize <= currentPos.length) {
                currentPos.length = ctx->lastPacketPayloadLength - frameHeaderSize;
            }
            else {
                if (ctx->lastPacketPayloadLength <= frameHeaderSize) {
                    Limelog("Invalid last payload length for header on frame %u: %u <= %u",
                            frameIndex, ctx->lastPacketPayloadLength, frameHeaderSize);
                }
                else {
                    Limelog("Invalid last payload length for packet size on frame %u: %u > %u",
                            frameIndex, ctx->lastPacketPayloadLength - frameHeaderSize, currentPos.length);
                }

                // Skip to the next frame and tell the host we lost this one
                ctx->decodingFrame = false;
                ctx->nextFrameNumber = frameIndex + 1;
                dropFrameState(ctx);
                if (ctx->waitingForIdrFrame) {
                    LiRequestIdrFrame();
                }
                else {
                    connectionDetectedFrameLoss(ctx->startFrameNumber, frameIndex);
                }

                return;
            }
        }

        // Other codecs are just passed through as is.
        queueFragment(ctx, existingEntry, currentPos.data, currentPos.offset, currentPos.length);
    }

    if (lastPacket) {
        // Move on to the next frame
        ctx->decodingFrame = false;
        ctx->nextFrameNumber = frameIndex + 1;

        // If we can't submit this frame due to a discontinuity in the bitstream,
        // inform the host (if needed) and drop the data.
        if (ctx->waitingForIdrFrame || ctx->waitingForRefInvalFrame) {
            // IDR wait takes priority over RFI wait (and an IDR frame will satisfy both)
            if (ctx->waitingForIdrFrame) {
                Limelog("Waiting for IDR frame\n");

                // We wait for the first fully received frame after a loss to approximate
                // detection of the recovery of the network. Requesting an IDR frame while
                // the network is unstable will just contribute to congestion collapse.
                if (ctx->waitingForNextSuccessfulFrame) {
                    LiRequestIdrFrame();
                }
            }
            else {
                // If we need an RFI frame first, then drop this frame
                // and update the reference frame invalidation window.
                Limelog("Waiting for RFI frame\n");
                connectionDetectedFrameLoss(ctx->startFrameNumber, frameIndex);
            }

            ctx->waitingForNextSuccessfulFrame = false;
            dropFrameState(ctx);
            return;
        }

        LC_ASSERT(!ctx->waitingForNextSuccessfulFrame);

        // Carry out any pending state drops. We can't just do this
        // arbitrarily in the middle of processing a frame because
        // may cause the depacketizer state to become corrupted. For
        // example, if we drop state after the first packet, the
        // depacketizer will next try to process a non-SOF packet,
        // and cause it to assert.
        if (ctx->dropStatePending) {
            if (ctx->nalChainHead && ctx->frameType == FRAME_TYPE_IDR) {
                // Don't drop the frame state if this frame is an IDR frame itself,
                // otherwise we'll lose this IDR frame without another in flight
                // and have to wait until we hit our consecutive drop limit to
                // request a new one (potentially several seconds).
                ctx->dropStatePending = false;
            }
            else {
                dropFrameState(ctx);
                return;
            }
        }

        reassembleFrame(ctx, frameIndex, extraFlags & NV_VIDEO_PACKET_EXTRA_FLAG_LTR_FRAME);
    }
}

// Called by the video RTP FEC queue to notify us of a lost frame
// if it determines the frame to be unrecoverable. This lets us
// avoid having to wait until the next received frame to determine
// that we lost a frame and submit an RFI request.
void notifyFrameLost(int streamIndex, unsigned int frameNumber, bool speculative) {
    VIDEO_DEPACKETIZER_CTX* ctx;

    if (!isValidStreamIndex(streamIndex)) {
        return;
    }
    ctx = &depacketizers[streamIndex];

    // We may not invalidate frames that we've already received
    LC_ASSERT(frameNumber >= ctx->startFrameNumber);

    // Drop state and determine if we need an IDR frame or if RFI is okay
    dropFrameState(ctx);

    // If dropFrameState() determined that RFI was usable, issue it now
    if (!ctx->waitingForIdrFrame) {
        LC_ASSERT(ctx->waitingForRefInvalFrame);

        if (speculative) {
            Limelog("Sending speculative RFI request for predicted loss of frame %d\n", frameNumber);
        }
        else {
            Limelog("Sending RFI request for unrecoverable frame %d\n", frameNumber);
        }

        // Advance the frame number since we won't be expecting this one anymore
        ctx->nextFrameNumber = frameNumber + 1;

        // Notify the host that we lost this one
        connectionDetectedFrameLoss(ctx->startFrameNumber, frameNumber);
    }
}

// Add an RTP Packet to the queue
void queueRtpPacket(int streamIndex, PRTPV_QUEUE_ENTRY queueEntryPtr) {
    VIDEO_DEPACKETIZER_CTX* ctx;
    int dataOffset;
    RTPV_QUEUE_ENTRY queueEntry = *queueEntryPtr;

    if (!isValidStreamIndex(streamIndex)) {
        return;
    }
    ctx = &depacketizers[streamIndex];

    LC_ASSERT(!queueEntry.isParity);
    LC_ASSERT(queueEntry.receiveTimeUs != 0);

    dataOffset = sizeof(*queueEntry.packet);
    if (queueEntry.packet->header & FLAG_EXTENSION) {
        dataOffset += 4; // 2 additional fields
    }

    // The packet length was validated by the RtpVideoQueue
    LC_ASSERT(queueEntry.length >= dataOffset + (int)sizeof(NV_VIDEO_PACKET));

    // Reuse the memory reserved for the RTPFEC_QUEUE_ENTRY to store the LENTRY_INTERNAL
    // now that we're in the depacketizer. We saved a copy of the real FEC queue entry
    // on the stack here so we can safely modify this memory in place.
    LC_ASSERT(sizeof(LENTRY_INTERNAL) <= sizeof(RTPV_QUEUE_ENTRY));
    PLENTRY_INTERNAL existingEntry = (PLENTRY_INTERNAL)queueEntryPtr;
    existingEntry->allocPtr = queueEntry.packet;

    processRtpPayload(ctx, (PNV_VIDEO_PACKET)(((char*)queueEntry.packet) + dataOffset),
                      queueEntry.length - dataOffset,
                      queueEntry.receiveTimeUs,
                      queueEntry.presentationTimeUs,
                      queueEntry.rtpTimestamp,
                      &existingEntry);

    if (existingEntry != NULL) {
        // processRtpPayload didn't want this packet, so just free it
        free(existingEntry->allocPtr);
    }
}

int LiGetPendingVideoFramesForStream(int streamIndex) {
    if (!isValidStreamIndex(streamIndex)) {
        return 0;
    }
    return LbqGetItemCount(&depacketizers[streamIndex].decodeUnitQueue);
}

int LiGetPendingVideoFrames(void) {
    return LiGetPendingVideoFramesForStream(0);
}
