# Raw USART encoder frame from the hand controller firmware.
# 16-byte packet: 0x55 0xAA | valid_mask(1) | 6x(lo, hi12) | checksum(1)
# Published already-assembled; decoding to radians happens offline
# (or in firmware, once calibration moves there).

std_msgs/Header header

uint16[6] raw         # 12-bit encoder counts, one per channel
uint8     valid_mask  # bit i => channel i is valid
