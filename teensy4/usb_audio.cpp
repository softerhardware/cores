/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2017 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <Arduino.h>
#include "usb_dev.h"
#include "usb_audio.h"
#include "debug/printf.h"

#ifdef AUDIO_INTERFACE

bool AudioInputUSB::update_responsibility;

struct usb_audio_features_struct AudioInputUSB::features = {0,0,FEATURE_MAX_VOLUME/2};

extern volatile uint8_t usb_high_speed;
static void rx_event(transfer_t *t);
static void tx_event(transfer_t *t);

/*static*/ transfer_t rx_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t sync_transfer __attribute__ ((used, aligned(32)));
/*static*/ transfer_t tx_transfer __attribute__ ((used, aligned(32)));
DMAMEM static uint8_t rx_buffer[AUDIO_RX_SIZE] __attribute__ ((aligned(32)));
DMAMEM static uint8_t tx_buffer[AUDIO_RX_SIZE] __attribute__ ((aligned(32)));
DMAMEM uint32_t usb_audio_sync_feedback __attribute__ ((aligned(32)));

uint8_t usb_audio_receive_setting=0;
uint8_t usb_audio_transmit_setting=0;
uint8_t usb_audio_sync_nbytes;
uint8_t usb_audio_sync_rshift;

uint32_t feedback_accumulator;

volatile uint32_t usb_audio_underrun_count;
volatile uint32_t usb_audio_overrun_count;

static void rx_event(transfer_t *t)
{
	if (t) {
		int len = AUDIO_RX_SIZE - ((rx_transfer.status >> 16) & 0x7FFF);
		printf("rx %u\n", len);
		usb_audio_receive_callback(len);
	}
	usb_prepare_transfer(&rx_transfer, rx_buffer, AUDIO_RX_SIZE, 0);
	arm_dcache_delete(&rx_buffer, AUDIO_RX_SIZE);
	usb_receive(AUDIO_RX_ENDPOINT, &rx_transfer);
}

static void sync_event(transfer_t *t)
{
	// USB 2.0 Specification, 5.12.4.2 Feedback, pages 73-75
	//printf("sync %x\n", sync_transfer.status); // too slow, can't print this much
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&sync_transfer, &usb_audio_sync_feedback, usb_audio_sync_nbytes, 0);
	arm_dcache_flush(&usb_audio_sync_feedback, usb_audio_sync_nbytes);
	usb_transmit(AUDIO_SYNC_ENDPOINT, &sync_transfer);
}

static void copy_to_buffers(const uint32_t *src, int16_t *left, int16_t *right, unsigned int len)
{
	uint32_t *target = (uint32_t*) src + len; 
	while ((src < target) && (((uintptr_t) left & 0x02) != 0)) {
		uint32_t n = *src++;
		*left++ = n & 0xFFFF;
		*right++ = n >> 16;
	}

	while ((src < target - 2)) {
		uint32_t n1 = *src++;
		uint32_t n = *src++;
		*(uint32_t *)left = (n1 & 0xFFFF) | ((n & 0xFFFF) << 16);
		left+=2;
		*(uint32_t *)right = (n1 >> 16) | ((n & 0xFFFF0000)) ;
		right+=2;
	}

	while ((src < target)) {
		uint32_t n = *src++;
		*left++ = n & 0xFFFF;
		*right++ = n >> 16;
	}
}





#if defined(USB_AUDIO_FEEDBACK_SOF)

// Precompute values for USB_AUDIO_FEEDBACK_SOF
// Total 0.125us to compute exponential moving average
static const uint32_t USB_AUDIO_FEEDBACK_SOF_MAX = 480000;

// Buffers
static const uint16_t USB_AUDIO_INPUT_BUFFERS=4;

// Limit for near under/over run to start adjusting clock
static const uint16_t USB_AUDIO_GUARD_RAIL=4;

#ifdef USB_AUDIO_48KHZ	
static const uint32_t USB_AUDIO_FEEDBACK_INIT = 805306368; // 48 * 2^24
static const uint32_t USB_AUDIO_FEEDBACK_MAX  = 805641912; // 48.020 * 2^24
static const uint32_t USB_AUDIO_FEEDBACK_MIN  = 804970824; // 47.980 * 2^24
#else
static const uint32_t USB_AUDIO_FEEDBACK_INIT = 739875226; // 44.1 * 2^24
static const uint32_t USB_AUDIO_FEEDBACK_MAX  = 740210769; // 44.120 * 2^24
static const uint32_t USB_AUDIO_FEEDBACK_MIN  = 739539681; // 44.080 * 2^24
#endif

// Static in this context (outside of a funciton) means the variable scope is this file only
static audio_block_t *input_left[USB_AUDIO_INPUT_BUFFERS];
static audio_block_t *input_right[USB_AUDIO_INPUT_BUFFERS];
static uint16_t incoming_index;
static uint16_t ready_index;

static uint16_t incoming_count;

static uint32_t usb_audio_near_overrun_count;
static uint32_t usb_audio_near_underrun_count;

static uint64_t usb_audio_samples_consumed;
static volatile uint32_t usb_audio_frames_counted;

void usb_audio_update_sof_count(void)
{
	if (usb_high_speed) usb_audio_frames_counted++;
	else usb_audio_frames_counted += 8; // full speed sof is 1ms or 8*0.125us
}

void usb_audio_configure(void)
{
	usb_audio_underrun_count = 0;
	usb_audio_overrun_count = 0;
	usb_audio_near_overrun_count = 0;
	usb_audio_near_underrun_count = 0;

	feedback_accumulator = USB_AUDIO_FEEDBACK_INIT;

	usb_audio_samples_consumed = 0;
	usb_audio_frames_counted = 0;

	for (uint16_t i = 0; i<USB_AUDIO_INPUT_BUFFERS; i++) {
		input_right[i] = NULL;
		input_left[i] = NULL;
	}
	incoming_count = 0;
	incoming_index = 0;
	ready_index = USB_AUDIO_INPUT_BUFFERS >> 1;

	if (usb_high_speed) {
		usb_audio_sync_nbytes = 4;
		usb_audio_sync_rshift = 8;
	} else {
		usb_audio_sync_nbytes = 3;
		usb_audio_sync_rshift = 10;
	}
	memset(&rx_transfer, 0, sizeof(rx_transfer));
	usb_config_rx_iso(AUDIO_RX_ENDPOINT, AUDIO_RX_SIZE, 1, rx_event);
	rx_event(NULL);
	memset(&sync_transfer, 0, sizeof(sync_transfer));
	usb_config_tx_iso(AUDIO_SYNC_ENDPOINT, usb_audio_sync_nbytes, 1, sync_event);
	sync_event(NULL);
	memset(&tx_transfer, 0, sizeof(tx_transfer));
	usb_config_tx_iso(AUDIO_TX_ENDPOINT, AUDIO_TX_SIZE, 1, tx_event);
	tx_event(NULL);
	usb_start_sof_interrupts(AUDIO_INTERFACE);
}

void AudioInputUSB::begin(void)
{
	// update_responsibility = update_setup();
	// TODO: update responsibility is tough, partly because the USB
	// interrupts aren't sychronous to the audio library block size,
	// but also because the PC may stop transmitting data, which
	// means we no longer get receive callbacks from usb.c
	update_responsibility = false;
}


// Called from the USB interrupt when an isochronous packet arrives
// we must completely remove it from the receive buffer before returning
//
void usb_audio_receive_callback(unsigned int len)
{
	unsigned int count, avail;
	audio_block_t *left, *right;
	const uint32_t *data;
	uint16_t next_incoming_index;

	len >>= 2; // 1 sample = 4 bytes: 2 left, 2 right
	data = (const uint32_t *)rx_buffer;

	count = incoming_count;
	left = input_left[incoming_index];
	right = input_right[incoming_index];
	if (left == NULL) {
		left = AudioStream::allocate();
		if (left == NULL) return;
		input_left[incoming_index] = left;
	}
	if (right == NULL) {
		right = AudioStream::allocate();
		if (right == NULL) return;
		input_right[incoming_index] = right;
	}

	while (len > 0) {

		next_incoming_index = incoming_index+1;
		if (next_incoming_index >= USB_AUDIO_INPUT_BUFFERS) {
			next_incoming_index = 0;
		}

		avail = AUDIO_BLOCK_SAMPLES - count;
		if (len < avail) {
			copy_to_buffers(data, left->data + count, right->data + count, len);
			incoming_count = count + len;

			// Check for near overrun condition
			if ((avail - len) >= USB_AUDIO_GUARD_RAIL) return;

			// Not enough available so check if next buffer occupied
			if (input_left[next_incoming_index] || input_right[next_incoming_index]) {
				usb_audio_near_overrun_count++;
				usb_audio_samples_consumed -= uint64_t(usb_audio_samples_consumed >> 25);
			}
			return;

		} else if (avail > 0) {
			copy_to_buffers(data, left->data + count, right->data + count, avail);
			data += avail;
			len -= avail;

			if (input_left[next_incoming_index] || input_right[next_incoming_index]) {
				// buffer overrun, PC sending too fast
				incoming_count = count + avail;
				if (len > 0) {
					usb_audio_overrun_count++;
					usb_audio_samples_consumed -= uint64_t(usb_audio_samples_consumed >> 25);

				} else {
					// Exactly enough space so near overrun
					usb_audio_near_overrun_count++;
					usb_audio_samples_consumed -= uint64_t(usb_audio_samples_consumed >> 25);
				}
				return;
			}
			

			send:

			incoming_index = next_incoming_index;
			left = AudioStream::allocate();
			if (left == NULL) {
				incoming_count = 0;
				return;
			}
			right = AudioStream::allocate();
			if (right == NULL) {
				AudioStream::release(left);
				incoming_count = 0;
				return;
			}

			input_left[incoming_index] = left;
			input_right[incoming_index] = right;
			count = 0;
		} else {

			if (input_left[next_incoming_index] || input_right[next_incoming_index]) {
				// Available is zero near overrun
				usb_audio_near_overrun_count++;
				return;
			}
			goto send; // recover from buffer overrun
		}
	}
	incoming_count = count;
}


void AudioInputUSB::update(void)
{
	audio_block_t *left, *right;
	static uint16_t rate_errors = 0;

	__disable_irq();

	//if (ready_index != incoming_index) {
		left = input_left[ready_index];
		input_left[ready_index] = NULL;
		right = input_right[ready_index];
		input_right[ready_index] = NULL;
	//} else {
	//	left = NULL;
	//	right = NULL;
	//}
	uint16_t next_ready_index;

	uint32_t frames_counted = usb_audio_frames_counted;
	uint16_t to_remove;
	// This is the frames to average over, exponential moving average
	if (frames_counted > USB_AUDIO_FEEDBACK_SOF_MAX) {
		to_remove = frames_counted - USB_AUDIO_FEEDBACK_SOF_MAX;
		usb_audio_frames_counted -= to_remove;
	} else {
		to_remove = 0;
	}

	__enable_irq();


#if 1
	//
	// This prints out the actual value of the feedback
	// correction (converted to Hz) and the min and max
	// value of incoming_count few times per second on
	// the console. This can be used to debug the
	// feedback control loop
	//
	static uint16_t debug=0;

	if (++debug > 1500) {
		debug=0;
		Serial.print("Corr= ");
		Serial.print(feedback_accumulator*0.00005960464477539063);
		Serial.print(" O");
		Serial.print(usb_audio_overrun_count);
		Serial.print(" NO");
		Serial.print(usb_audio_near_overrun_count);
		Serial.print(" U");
		Serial.print(usb_audio_underrun_count);
		Serial.print(" NU");
		Serial.println(usb_audio_near_underrun_count);
	}
#endif

	// Hope the compiler optimizes this to a constant
	usb_audio_samples_consumed += (uint64_t(AUDIO_BLOCK_SAMPLES) << 27);
	feedback_accumulator = usb_audio_samples_consumed / frames_counted;
	usb_audio_samples_consumed -= uint64_t(uint64_t(feedback_accumulator) * uint32_t(to_remove));

	// Check for out of range rates
	if ((feedback_accumulator > USB_AUDIO_FEEDBACK_MAX) || (feedback_accumulator < USB_AUDIO_FEEDBACK_MIN)) {
		rate_errors++;
	}

	// Reset if sufficiently out of range
	if (rate_errors > 500) {
		__disable_irq();
		feedback_accumulator = USB_AUDIO_FEEDBACK_INIT;
		usb_audio_frames_counted = 0;
		usb_audio_samples_consumed = 0;
		__enable_irq();
		rate_errors = 0;
	}

	next_ready_index = ready_index+1;
	if (next_ready_index >= USB_AUDIO_INPUT_BUFFERS) {
		next_ready_index = 0;
	}

	if (!left || !right) {
		usb_audio_underrun_count++;
		// Don't adjust samples here as when there is no audio for host there will be underruns
	} else if (next_ready_index == incoming_index) {
		if (input_left[next_ready_index] || input_right[next_ready_index]) {
			// Next buffers are being filled, check incoming count
			if (incoming_count <= USB_AUDIO_GUARD_RAIL) {
				usb_audio_near_underrun_count++;
				// Local clock is running faster so speed up relation
				usb_audio_samples_consumed += uint64_t(usb_audio_samples_consumed >> 25);
			}
		} else {
			// Next buffers are not even being filled
			usb_audio_near_underrun_count++;
			usb_audio_samples_consumed += uint64_t(usb_audio_samples_consumed >> 25);
		}
	}

	if (left) {
		ready_index = next_ready_index;
		transmit(left, 0);
		release(left);
	}
	if (right) {
		transmit(right, 1);
		release(right);
	}
}



#else

audio_block_t * AudioInputUSB::incoming_left;
audio_block_t * AudioInputUSB::incoming_right;
audio_block_t * AudioInputUSB::ready_left;
audio_block_t * AudioInputUSB::ready_right;
uint16_t AudioInputUSB::incoming_count;
uint8_t AudioInputUSB::receive_flag;

#if defined(USB_AUDIO_FEEDBACK_DL1YCF)
// DL1YCF:
// two new variables for the "damped oscillator"
// feedback loop
int32_t feedback_speed_correction;   // buffer for the speed correction
uint16_t old_incoming_count;         // measuring the speed
#endif

void usb_audio_configure(void)
{
	printf("usb_audio_configure\n");
	usb_audio_underrun_count = 0;
	usb_audio_overrun_count = 0;

#ifdef USB_AUDIO_48KHZ
	feedback_accumulator = 805306368; // 48 * 2^24
#else
	feedback_accumulator = 739875226; // 44.1 * 2^24
#endif

#if defined(USB_AUDIO_FEEDBACK_DL1YCF)
	// DL1YCF: init the two new variables
	feedback_speed_correction = 0;
	old_incoming_count=0;
#endif

	if (usb_high_speed) {
		usb_audio_sync_nbytes = 4;
		usb_audio_sync_rshift = 8;
	} else {
		usb_audio_sync_nbytes = 3;
		usb_audio_sync_rshift = 10;
	}
	memset(&rx_transfer, 0, sizeof(rx_transfer));
	usb_config_rx_iso(AUDIO_RX_ENDPOINT, AUDIO_RX_SIZE, 1, rx_event);
	rx_event(NULL);
	memset(&sync_transfer, 0, sizeof(sync_transfer));
	usb_config_tx_iso(AUDIO_SYNC_ENDPOINT, usb_audio_sync_nbytes, 1, sync_event);
	sync_event(NULL);
	memset(&tx_transfer, 0, sizeof(tx_transfer));
	usb_config_tx_iso(AUDIO_TX_ENDPOINT, AUDIO_TX_SIZE, 1, tx_event);
	tx_event(NULL);
	usb_start_sof_interrupts(AUDIO_INTERFACE);
}

void AudioInputUSB::begin(void)
{
	incoming_count = 0;
	incoming_left = NULL;
	incoming_right = NULL;
	ready_left = NULL;
	ready_right = NULL;
	receive_flag = 0;
	// update_responsibility = update_setup();
	// TODO: update responsibility is tough, partly because the USB
	// interrupts aren't sychronous to the audio library block size,
	// but also because the PC may stop transmitting data, which
	// means we no longer get receive callbacks from usb.c
	update_responsibility = false;
}


// Called from the USB interrupt when an isochronous packet arrives
// we must completely remove it from the receive buffer before returning
//
void usb_audio_receive_callback(unsigned int len)
{
	unsigned int count, avail;
	audio_block_t *left, *right;
	const uint32_t *data;

	AudioInputUSB::receive_flag = 1;
	len >>= 2; // 1 sample = 4 bytes: 2 left, 2 right
	data = (const uint32_t *)rx_buffer;

	count = AudioInputUSB::incoming_count;
	left = AudioInputUSB::incoming_left;
	right = AudioInputUSB::incoming_right;
	if (left == NULL) {
		left = AudioStream::allocate();
		if (left == NULL) return;
		AudioInputUSB::incoming_left = left;
	}
	if (right == NULL) {
		right = AudioStream::allocate();
		if (right == NULL) return;
		AudioInputUSB::incoming_right = right;
	}
	while (len > 0) {
		avail = AUDIO_BLOCK_SAMPLES - count;
		if (len < avail) {
			copy_to_buffers(data, left->data + count, right->data + count, len);
			AudioInputUSB::incoming_count = count + len;
			return;
		} else if (avail > 0) {
			copy_to_buffers(data, left->data + count, right->data + count, avail);
			data += avail;
			len -= avail;
			if (AudioInputUSB::ready_left || AudioInputUSB::ready_right) {
				// buffer overrun, PC sending too fast
				AudioInputUSB::incoming_count = count + avail;
				if (len > 0) {
					usb_audio_overrun_count++;
#if defined(USB_AUDIO_FEEDBACK_DL1YCF)
					// the measured speed will be too low if we
					// do not account for "ignored" incoming samples
					old_incoming_count -= len;
#endif
					//Serial.print("U");
					//Serial.println(len);
					//printf("!");
					//serial_phex(len);
				}
				return;
			}
			send:
			AudioInputUSB::ready_left = left;
			AudioInputUSB::ready_right = right;
			//if (AudioInputUSB::update_responsibility) AudioStream::update_all();
			left = AudioStream::allocate();
			if (left == NULL) {
				AudioInputUSB::incoming_left = NULL;
				AudioInputUSB::incoming_right = NULL;
				AudioInputUSB::incoming_count = 0;
				return;
			}
			right = AudioStream::allocate();
			if (right == NULL) {
				AudioStream::release(left);
				AudioInputUSB::incoming_left = NULL;
				AudioInputUSB::incoming_right = NULL;
				AudioInputUSB::incoming_count = 0;
				return;
			}
			AudioInputUSB::incoming_left = left;
			AudioInputUSB::incoming_right = right;
			count = 0;
		} else {
			if (AudioInputUSB::ready_left || AudioInputUSB::ready_right) return;
			goto send; // recover from buffer overrun
		}
	}
	AudioInputUSB::incoming_count = count;
}

void AudioInputUSB::update(void)
{
	audio_block_t *left, *right;

	__disable_irq();
	left = ready_left;
	ready_left = NULL;
	right = ready_right;
	ready_right = NULL;
	uint16_t c = incoming_count;
	uint8_t f = receive_flag;

#if defined(USB_AUDIO_FEEDBACK_DL1YCF)
	uint16_t d = old_incoming_count;
	old_incoming_count = incoming_count;
#endif

	receive_flag = 0;
	__enable_irq();


#if 1
	//
	// This prints out the actual value of the feedback
	// correction (converted to Hz) and the min and max
	// value of incoming_count few times per second on
	// the console. This can be used to debug the
	// feedback control loop
	//
	static uint16_t max_buf=0;
	static uint16_t min_buf=9999;
	static uint16_t debug=0;

	if (c > max_buf) max_buf=c;
	if (c < min_buf) min_buf=c;

	if (++debug > 1500) {
		debug=0;
		Serial.print("Corr= ");
		Serial.print(feedback_accumulator*0.00005960464477539063);
#if defined(USB_AUDIO_FEEDBACK_DL1YCF)
		Serial.print(" FSC=");
		Serial.print(feedback_speed_correction);
#endif
		Serial.print(" Min= ");
		Serial.print(min_buf); 
		Serial.print(" Max= ");
		Serial.print(max_buf);
		Serial.print(" O");
		Serial.print(usb_audio_overrun_count);
		Serial.print(" U");
		Serial.print(usb_audio_underrun_count);
		Serial.println(";");
		min_buf=9999;
		max_buf=0;
	}
#endif

#if defined(USB_AUDIO_FEEDBACK_DL1YCF)

	if (f) {
		//
		// DL1YCF: new feedback "damped oscillator" correction
		//
		feedback_accumulator += AUDIO_BLOCK_SAMPLES/2 - (int) c;
		//
		// The two "magic" constants 2508 and 2400 correspond to
		// to a damping ratio of zeta=0.5. To increase zeta to
		// the "critically damped" case (zeta=1.0),
		// these two parameters must be doubled.
		//
#ifdef USB_AUDIO_48KHZ
		feedback_speed_correction -= 2508*((int) c - (int) d);
#else
		feedback_speed_correction -= 2400*((int) c - (int) d);
#endif
		//
		// "stretch" and average the updates over a long period
		// to filter out high-frequency oscillations in the incoming
		// buffer count
		//
		if (feedback_speed_correction < -64) {
			feedback_accumulator -=64;
			feedback_speed_correction += 64;
		} else if (feedback_speed_correction > 64) {
			feedback_accumulator += 64;
			feedback_speed_correction -=64;
 		}
		if (!left || !right) {
			usb_audio_underrun_count++;
			//
			// The buffer filling will bump up by one block
			// since we did not "fetch" the data.
			// This will compromise the speed measurement.
			//
			__disable_irq();
			old_incoming_count += AUDIO_BLOCK_SAMPLES;
			__enable_irq();
		}
	}

#else
	if (f) {
		int diff = AUDIO_BLOCK_SAMPLES/2 - (int)c;
		feedback_accumulator += diff * 1;
		//uint32_t feedback = (feedback_accumulator >> 8) + diff * 100;
		//usb_audio_sync_feedback = feedback;

		//printf(diff >= 0 ? "." : "^");
	}
	//serial_phex(c);
	//serial_print(".");

	if (!left || !right) {
		usb_audio_underrun_count++;
		//Serial.print("O");
		//Serial.println(incoming_count);
		//printf("#"); // buffer underrun - PC sending too slow
		// USB_AUDIO_48KHZ For some reason there are many underruns during the 1 second of powerup
		// It seems as if the PC is either sending too short packets, or too infrequent packets at first
		// The line below causes the feedback_accumulator to be way off
		if (f) feedback_accumulator += 3500;
	}
#endif

	if (left) {
		transmit(left, 0);
		release(left);
	}
	if (right) {
		transmit(right, 1);
		release(right);
	}
}



#endif // feedback type








#if 1
bool AudioOutputUSB::update_responsibility;
audio_block_t * AudioOutputUSB::left_1st;
audio_block_t * AudioOutputUSB::left_2nd;
audio_block_t * AudioOutputUSB::right_1st;
audio_block_t * AudioOutputUSB::right_2nd;
uint16_t AudioOutputUSB::offset_1st;

/*DMAMEM*/ uint16_t usb_audio_transmit_buffer[AUDIO_TX_SIZE/2] __attribute__ ((used, aligned(32)));


static void tx_event(transfer_t *t)
{
	int len = usb_audio_transmit_callback();
	usb_audio_sync_feedback = feedback_accumulator >> usb_audio_sync_rshift;
	usb_prepare_transfer(&tx_transfer, usb_audio_transmit_buffer, len, 0);
	arm_dcache_flush_delete(usb_audio_transmit_buffer, len);
	usb_transmit(AUDIO_TX_ENDPOINT, &tx_transfer);
}


void AudioOutputUSB::begin(void)
{
	update_responsibility = false;
	left_1st = NULL;
	right_1st = NULL;
}

static void copy_from_buffers(uint32_t *dst, int16_t *left, int16_t *right, unsigned int len)
{
	// TODO: optimize...
	while (len > 0) {
		*dst++ = (*right++ << 16) | (*left++ & 0xFFFF);
		len--;
	}
}

void AudioOutputUSB::update(void)
{
	audio_block_t *left, *right;

	// TODO: we shouldn't be writing to these......
	//left = receiveReadOnly(0); // input 0 = left channel
	//right = receiveReadOnly(1); // input 1 = right channel
	left = receiveWritable(0); // input 0 = left channel
	right = receiveWritable(1); // input 1 = right channel
	if (usb_audio_transmit_setting == 0) {
		if (left) release(left);
		if (right) release(right);
		if (left_1st) { release(left_1st); left_1st = NULL; }
		if (left_2nd) { release(left_2nd); left_2nd = NULL; }
		if (right_1st) { release(right_1st); right_1st = NULL; }
		if (right_2nd) { release(right_2nd); right_2nd = NULL; }
		offset_1st = 0;
		return;
	}
	if (left == NULL) {
		left = allocate();
		if (left == NULL) {
			if (right) release(right);
			return;
		}
		memset(left->data, 0, sizeof(left->data));
	}
	if (right == NULL) {
		right = allocate();
		if (right == NULL) {
			release(left);
			return;
		}
		memset(right->data, 0, sizeof(right->data));
	}
	__disable_irq();
	if (left_1st == NULL) {
		left_1st = left;
		right_1st = right;
		offset_1st = 0;
	} else if (left_2nd == NULL) {
		left_2nd = left;
		right_2nd = right;
	} else {
		// buffer overrun - PC is consuming too slowly
		audio_block_t *discard1 = left_1st;
		left_1st = left_2nd;
		left_2nd = left;
		audio_block_t *discard2 = right_1st;
		right_1st = right_2nd;
		right_2nd = right;
		offset_1st = 0; // TODO: discard part of this data?
		//serial_print("*");
		release(discard1);
		release(discard2);
	}
	__enable_irq();
}


// Called from the USB interrupt when ready to transmit another
// isochronous packet.  If we place data into the transmit buffer,
// the return is the number of bytes.  Otherwise, return 0 means
// no data to transmit
unsigned int usb_audio_transmit_callback(void)
{

	uint32_t avail, num, target, offset, len=0;
	audio_block_t *left, *right;

#ifdef USB_AUDIO_48KHZ
	target = 48;
#else
	static uint32_t count=5;
	if (++count < 10) {   // TODO: dynamic adjust to match USB rate
		target = 44;
	} else {
		count = 0;
		target = 45;
	}
#endif

	while (len < target) {
		num = target - len;
		left = AudioOutputUSB::left_1st;
		if (left == NULL) {
			// buffer underrun - PC is consuming too quickly
			memset(usb_audio_transmit_buffer + len, 0, num * 4);
			//serial_print("%");
			break;
		}
		right = AudioOutputUSB::right_1st;
		offset = AudioOutputUSB::offset_1st;

		avail = AUDIO_BLOCK_SAMPLES - offset;
		if (num > avail) num = avail;

		copy_from_buffers((uint32_t *)usb_audio_transmit_buffer + len,
			left->data + offset, right->data + offset, num);
		len += num;
		offset += num;
		if (offset >= AUDIO_BLOCK_SAMPLES) {
			AudioStream::release(left);
			AudioStream::release(right);
			AudioOutputUSB::left_1st = AudioOutputUSB::left_2nd;
			AudioOutputUSB::left_2nd = NULL;
			AudioOutputUSB::right_1st = AudioOutputUSB::right_2nd;
			AudioOutputUSB::right_2nd = NULL;
			AudioOutputUSB::offset_1st = 0;
		} else {
			AudioOutputUSB::offset_1st = offset;
		}
	}
	return target * 4;
}
#endif




struct setup_struct {
  union {
    struct {
	uint8_t bmRequestType;
	uint8_t bRequest;
	union {
		struct {
			uint8_t bChannel;  // 0=main, 1=left, 2=right
			uint8_t bCS;       // Control Selector
		};
		uint16_t wValue;
	};
	union {
		struct {
			uint8_t bIfEp;     // type of entity
			uint8_t bEntityId; // UnitID, TerminalID, etc.
		};
		uint16_t wIndex;
	};
	uint16_t wLength;
    };
  };
};

int usb_audio_get_feature(void *stp, uint8_t *data, uint32_t *datalen)
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0xA1) { // should check bRequest, bChannel, and UnitID
			if (setup.bCS==0x01) { // mute
				data[0] = AudioInputUSB::features.mute;  // 1=mute, 0=unmute
				*datalen = 1;
				return 1;
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x81) { // GET_CURR
					data[0] = AudioInputUSB::features.volume & 0xFF;
					data[1] = (AudioInputUSB::features.volume>>8) & 0xFF;
				}
				else if (setup.bRequest==0x82) { // GET_MIN
					//serial_print("vol get_min\n");
					data[0] = 0;     // min level is 0
					data[1] = 0;
				}
				else if (setup.bRequest==0x83) { // GET_MAX
					data[0] = FEATURE_MAX_VOLUME;  // max level, for range of 0 to MAX
					data[1] = 0;
				}
				else if (setup.bRequest==0x84) { // GET_RES
					data[0] = 1; // increment vol by by 1
					data[1] = 0;
				}
				else { // pass over SET_MEM, etc.
					return 0;
				}
				*datalen = 2;
				return 1;
			}
	}
	return 0;
}

int usb_audio_set_feature(void *stp, uint8_t *buf) 
{
	struct setup_struct setup = *((struct setup_struct *)stp);
	if (setup.bmRequestType==0x21) { // should check bRequest, bChannel and UnitID
			if (setup.bCS==0x01) { // mute
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSB::features.mute = buf[0]; // 1=mute,0=unmute
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
			else if (setup.bCS==0x02) { // volume
				if (setup.bRequest==0x01) { // SET_CUR
					AudioInputUSB::features.volume = buf[0];
					AudioInputUSB::features.change = 1;
					return 1;
				}
			}
	}
	return 0;
}



#endif // AUDIO_INTERFACE
