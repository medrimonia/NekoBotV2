#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <wirish/wirish.h>
#include "terminal.h"
#include "dxl.h"

volatile static bool initialized = false;
volatile static unsigned int dxl_timeout;
struct dxl_packet incoming_packet;

void dxl_packet_init(struct dxl_packet *packet)
{
    packet->dxl_state = 0;
    packet->process = false;
}

/**
 * Writes the given packet to the buffer
 */
int dxl_write_packet(struct dxl_packet *packet, ui8 *buffer)
{
    int i;
    unsigned int pos = 0;

    buffer[pos++] = 0xff;
    buffer[pos++] = 0xff;
    buffer[pos++] = packet->id;
    buffer[pos++] = packet->parameter_nb+2;
    buffer[pos++] = packet->instruction;

    for (i=0; i<packet->parameter_nb; i++) {
        buffer[pos++] = packet->parameters[i];
    }

    buffer[pos++] = dxl_compute_checksum(packet);

    return pos;
}

void dxl_copy_packet(struct dxl_packet *from, struct dxl_packet *to)
{
    memcpy((void *)to, (void *)from, sizeof(struct dxl_packet));
}

ui8 dxl_compute_checksum(struct dxl_packet *packet) {
    int i;
    unsigned int sum = 0;

    sum += packet->id;
    sum += packet->instruction;
    sum += packet->parameter_nb+2;
    
    for (i=0; i<packet->parameter_nb; i++) {
        sum += packet->parameters[i];
    }

    sum = ~(sum & 0xFF);
    
    return (ui8) sum;
}

void dxl_packet_push_byte(struct dxl_packet *packet, ui8 b)
{
    switch (packet->dxl_state) {
        case 0:
        case 1:
            if (b != 0xFF) {
                goto pc_error;
            }
            break;
        case 2:
            packet->id = b;
            break;
        case 3:
            packet->parameter_nb = b - 2;
            break;
        case 4:
            packet->instruction = b;
            break;
        default:
            if (packet->dxl_state - 4 > packet->parameter_nb) {
                goto pc_ended;

            } else {
                packet->parameters[packet->dxl_state - 5] = b;
            }

            if (packet->dxl_state - 4 > DXL_MAX_PARAMS) {
                goto pc_error;
            }
    }
        
    packet->dxl_state++;
    return;

pc_ended:
    if (dxl_compute_checksum(packet) == b) {
        packet->process = true;
    }

    packet->dxl_state = 0;
    return;
pc_error:
    packet->dxl_state = 0;
}

void dxl_init(int baudrate)
{
#if defined(BOARD_opencm904)
    dxl_timeout = 3000000/baudrate;
    initialized = true;
    afio_remap(AFIO_REMAP_USART1);

    // Initializing pins
    gpio_set_mode(GPIOB, 6, GPIO_AF_OUTPUT_PP);
    gpio_set_mode(GPIOB, 7, GPIO_INPUT_FLOATING);

    // Direction pins
    pinMode(DXL_DIRECTION, OUTPUT);
    digitalWrite(DXL_DIRECTION, LOW);

    DXL_DEVICE.begin(baudrate);
#endif
}

// Sends a packet to the dynamixel bus
void dxl_send(struct dxl_packet *packet)
{
    if (!initialized) {
        return;
    }

    ui8 buffer[DXL_BUFFER_SIZE];
    int n = dxl_write_packet(packet, buffer);

    digitalWrite(DXL_DIRECTION, HIGH); // TX
    asm("nop");
    DXL_DEVICE.write(buffer, n);
    DXL_DEVICE.waitDataToBeSent();
    asm("nop");
    digitalWrite(DXL_DIRECTION, LOW); // RX

    incoming_packet.process = false;
}

// Wait and get the reply of a request
struct dxl_packet *dxl_get_reply()
{
    if (!initialized) {
        return NULL;
    }

    int now = millis();

    while ((millis()-now) < dxl_timeout) {
        dxl_tick();
        if (incoming_packet.process) {
            return &incoming_packet;
        }
    }

    // Timeout without any reply
    return NULL;
}

// Sends a request and get the reply
struct dxl_packet *dxl_send_reply(struct dxl_packet *request)
{
    dxl_send(request);

    return dxl_get_reply();
}

// Tick, reading the incoming packet from the dynamixel device
void dxl_tick()
{
    if (initialized) {
        while (DXL_DEVICE.available()) {
            ui8 c = DXL_DEVICE.read();
            dxl_packet_push_byte(&incoming_packet, c);
        }
    }
}

// Forwarding USB to Dynamixel
void dxl_forward()
{
    while (true && initialized) {
        struct dxl_packet current_packet;

        // Receiving packets
        dxl_tick();
        if (incoming_packet.process) {
            incoming_packet.process = false;
            ui8 buffer[DXL_BUFFER_SIZE];
            int n = dxl_write_packet(&incoming_packet, buffer);
            SerialUSB.write(buffer, n);
        }

        // Sending packets
        while (SerialUSB.available()) {
            dxl_packet_push_byte(&current_packet, SerialUSB.read());

            if (current_packet.process) {
                current_packet.process = false;
                dxl_send(&current_packet);
            }
        }
    }
}

// Pings a servo and returns true if it's alive
bool dxl_ping(ui8 id)
{
    struct dxl_packet request;
    request.id = id;
    request.instruction = DXL_CMD_PING;
    request.parameter_nb = 0;
    
    return (dxl_send_reply(&request)!=NULL);
}
