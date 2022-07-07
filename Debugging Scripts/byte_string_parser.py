s = b"\xff\xff\xff\xff\xfe\x03\tURControl\x05\x0b\x00\x00\x00\x00\x00\x00\x00\x0019-05-2021, 06:19:30"

i = 0

iostring = ""

while i < len(s):
    if s[i] == "\\":
