# One CAN message, matching the comms library (comms.h, currently in firmflake common libs).
#
# sender_id   - 13 bit device id of whoever sent the message (see device_ids.h).
#               Filled in by the sender, preserved on the way out of the bus.
# type        - 16 bit message type id (msg_type_t in comms.h).
# payload_len - number of valid bytes in payload. Same meaning as
#               encoded_msg_t::payload_len.
# payload     - raw encoded payload bytes, exactly what encode() produced.
#               Variable length so this message also works for CAN FD later,
#               but on classic CAN it is never more than 8 bytes.

uint16 sender_id
uint16 type
uint8 payload_len
uint8[] payload