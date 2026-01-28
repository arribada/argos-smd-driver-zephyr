#!/usr/bin/env python3
"""
Convert a binary firmware file to a C array for OTA testing

Usage: python3 fw_to_array.py <input.bin> <output.h>
"""

import sys
import os

def bin_to_c_array(input_file, output_file):
    """Convert binary file to C array"""

    # Read binary file
    with open(input_file, 'rb') as f:
        data = f.read()

    # Get filename without extension
    basename = os.path.basename(input_file)
    array_name = os.path.splitext(basename)[0].replace('-', '_').replace('.', '_')

    # Generate C header file
    with open(output_file, 'w') as f:
        f.write(f"/* Auto-generated from {basename} */\n")
        f.write(f"/* File size: {len(data)} bytes */\n\n")
        f.write(f"#ifndef FW_IMAGE_H\n")
        f.write(f"#define FW_IMAGE_H\n\n")
        f.write(f"#include <stdint.h>\n\n")

        # Write array
        f.write(f"static const uint8_t firmware_image[] = {{\n")

        # Write data in rows of 12 bytes
        for i in range(0, len(data), 12):
            chunk = data[i:i+12]
            hex_values = ', '.join(f'0x{b:02X}' for b in chunk)
            f.write(f"\t{hex_values},\n")

        f.write("};\n\n")
        f.write(f"#define FIRMWARE_IMAGE_SIZE {len(data)}\n\n")
        f.write(f"#endif /* FW_IMAGE_H */\n")

    print(f"Generated {output_file} with {len(data)} bytes")

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 fw_to_array.py <input.bin> <output.h>")
        sys.exit(1)

    input_file = sys.argv[1]
    output_file = sys.argv[2]

    if not os.path.exists(input_file):
        print(f"Error: Input file '{input_file}' not found")
        sys.exit(1)

    bin_to_c_array(input_file, output_file)
