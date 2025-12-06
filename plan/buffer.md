# Frame Buffer Refactor: Zero-Copy Pool + Queue

## Overview

Replace the current manual ring buffer implementation with a zero-copy buffer pool using FreeRTOS queues. This provides thread-safety, efficient blocking, and minimal data copying.

## Current Implementation

**File**: `components/led/led.c`

- Manual circular buffer with `_head` / `_tail` indices
- `FRAME_t _frame_buffer[CONFIG_LED_FRAME_BUFFER_SIZE]` - static array of frames
- `fifo_peek()` / `fifo_read()` / `fifo_write()` - manual FIFO operations
- No thread synchronization between MQTT producer and LED consumer
- Busy-waiting with `vTaskDelay(0)` when buffer is empty

## Proposed Implementation

### Architecture

```
┌─────────────────┐     ┌─────────────────┐     ┌─────────────────┐
│   Free Queue    │────▶│     Producer    │────▶│   Ready Queue   │
│  (empty bufs)   │     │  (MQTT context) │     │ (filled frames) │
└─────────────────┘     └─────────────────┘     └─────────────────┘
        ▲                                                │
        │                                                ▼
        │               ┌─────────────────┐              │
        └───────────────│    Consumer     │◀─────────────┘
                        │   (LED task)    │
                        └─────────────────┘
```

### Data Structures

```c
// Existing - no changes
static FRAME_t _frame_pool[CONFIG_LED_FRAME_BUFFER_SIZE];

// New - replace head/tail with queues
static QueueHandle_t _free_queue;   // Pointers to available buffers
static QueueHandle_t _ready_queue;  // Pointers to frames ready to display
```

### Flow

1. **Initialization**: All buffer pointers pushed to `_free_queue`
2. **Producer** (`led_push_stream`):
   - Pop pointer from `_free_queue` (non-blocking)
   - If none available, drop frame and return false
   - Copy incoming data to buffer
   - Push pointer to `_ready_queue`
3. **Consumer** (`led_task`):
   - Block on `_ready_queue` until frame available
   - Process frame (ws2811_setColors, ack callback)
   - Return pointer to `_free_queue`

## Implementation Steps

### Step 1: Add queue handles and remove old globals

Remove:
- `uint8_t _head`
- `uint8_t _tail`

Add:
- `QueueHandle_t _free_queue`
- `QueueHandle_t _ready_queue`

Rename for clarity:
- `_frame_buffer` → `_frame_pool`

### Step 2: Remove old FIFO functions

Delete:
- `fifo_peek()`
- `fifo_read()`
- `fifo_write()`

### Step 3: Update `led_initialise()`

```c
void led_initialise(led_log log_callback, led_ack ack_callback, int *gpios, size_t count) {
    _running = true;
    _log_callback = log_callback;
    _ack_callback = ack_callback;

    // Create queues for pointer passing
    _free_queue = xQueueCreate(CONFIG_LED_FRAME_BUFFER_SIZE, sizeof(FRAME_t*));
    _ready_queue = xQueueCreate(CONFIG_LED_FRAME_BUFFER_SIZE, sizeof(FRAME_t*));

    // Populate free queue with all buffer pointers
    for (int i = 0; i < CONFIG_LED_FRAME_BUFFER_SIZE; i++) {
        FRAME_t *ptr = &_frame_pool[i];
        xQueueSend(_free_queue, &ptr, 0);
    }

    ws2811_init(gpios, count);
}
```

### Step 4: Update `led_push_stream()`

```c
uint8_t led_push_stream(char *data) {
    FRAME_t *buf;

    // Try to get a free buffer (non-blocking)
    if (xQueueReceive(_free_queue, &buf, 0) != pdTRUE) {
        // No free buffers available - drop frame
        _dropCount++;
        if (_force_log_drop || _log) {
            char msg[60];
            sprintf(msg, "BUF DROP (S:%d)", _seq);
            _log_callback(msg);
            _seq++;
        }
        return false;
    }

    // Copy frame data into buffer
    memcpy(buf, data, sizeof(FRAME_t));

    if (_log) {
        char msg[60];
        sprintf(msg, "BUF W (A:%d, S:%d)", buf->ackID, _seq);
        _log_callback(msg);
        _seq++;
    }

    // Send to ready queue
    xQueueSend(_ready_queue, &buf, 0);
    return true;
}
```

### Step 5: Update `led_task()`

```c
void led_task(void *pParam) {
    FRAME_t *frame;

    _sampling_start = esp_timer_get_time();

    while (true) {
        // Log drop rate periodically
        int64_t delta = esp_timer_get_time() - _sampling_start;
        if (delta > 1000000) {
            float fps = (float)_dropCount / ((float)delta / 1000000.0f);
            char msg[60];
            sprintf(msg, "BUF DROP FPS %.1f (S:%d)", fps, _seq);
            _log_callback(msg);
            _seq++;

            _dropCount = 0;
            _sampling_start = esp_timer_get_time();
        }

        if (_running) {
            // Block until frame available (with timeout for drop rate logging)
            if (xQueueReceive(_ready_queue, &frame, pdMS_TO_TICKS(100)) == pdTRUE) {
                ws2811_setColors(frame->len, frame->data);
                _ack_callback(frame->ackID);

                if (_log) {
                    char msg[60];
                    sprintf(msg, "BUF R (A:%d, S:%d)", frame->ackID, _seq);
                    _log_callback(msg);
                    _seq++;
                }

                // Return buffer to free pool
                xQueueSend(_free_queue, &frame, 0);
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
}
```

## Benefits

| Aspect | Before | After |
|--------|--------|-------|
| Thread safety | None (race conditions possible) | FreeRTOS queue handles synchronization |
| Consumer idle | Busy-wait with `vTaskDelay(0)` | Efficient block on `xQueueReceive` |
| Queue operations | Manual index math | Proven FreeRTOS primitives |
| Data copied through queue | Full `FRAME_t` struct | 4-byte pointer only |
| Code complexity | ~70 lines of FIFO code | ~10 lines of queue setup |

## Logging Changes

The current logging includes head/tail indices which won't exist after refactoring. Update log messages to show queue depths instead:

```c
// Optional: show queue fill levels
UBaseType_t free_count = uxQueueMessagesWaiting(_free_queue);
UBaseType_t ready_count = uxQueueMessagesWaiting(_ready_queue);
sprintf(msg, "BUF W (A:%d, F:%d, R:%d, S:%d)", buf->ackID, free_count, ready_count, _seq);
```

## Testing

1. Verify frame throughput matches or exceeds previous implementation
2. Confirm no dropped frames under normal load
3. Verify graceful degradation under overload (frames dropped, no crash)
4. Check memory usage (should be similar - same pool size)
5. Confirm LED output timing remains consistent

## Optional Future Improvements

- Add `xQueueReset()` call to flush buffers on stop/start
- Consider `xQueueOverwrite()` for "latest frame only" mode
- Add queue high-water mark tracking via `uxQueueSpacesAvailable()`
