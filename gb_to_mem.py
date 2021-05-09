# Run in command line: python3 gb_to_mem.py <input.gb> <output.mem>
# Author: Fadi Alzammar

import sys

with open(sys.argv[1], 'rb') as f_in:
    binary = f_in.read()

hex_list = ['{:02x}'.format(c) for c in binary]

hex_str = ''
i = 0

for opcode in hex_list:
    hex_str += opcode + " \n"

with open(sys.argv[2], 'w') as f_out:
    f_out.write(hex_str)