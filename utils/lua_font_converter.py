import re
import sys
import os

def convert_binary_to_hex_byte(binary_list):
    """
    Converts a list of 8 binary digits (0 or 1) into a hexadecimal byte string.
    Example: [0, 1, 1, 1, 1, 1, 1, 0] -> '0x7E'
    """
    if len(binary_list) != 8:
        # Handle cases where a row doesn't have exactly 8 bits, though unlikely for 8x8 font
        raise ValueError("Input list must contain exactly 8 elements.")

    # Convert the list of integers into a single binary string
    binary_str = "".join(map(str, binary_list))

    # Convert the binary string to an integer, then to a hex string
    # The format '0x%02X' ensures the output is always 2 digits (e.g., 0x0A instead of 0xA)
    return '0x{:02X}'.format(int(binary_str, 2))


def convert_lua_font_to_c_array(lua_font_string):
    """
    Parses the Lua font definition string and converts it into a C-style
    static array definition of 8-byte characters, applying a 90-degree
    counter-clockwise rotation to each character.
    """
    # 1. Regex to find all character blocks: [index] = { ... data ... },
    # The pattern captures the content inside the innermost braces {}.
    # re.DOTALL is crucial to match across multiple lines.
    char_blocks = re.findall(r'\[\s*\d+\s*\]\s*=\s*\{([^}]+)\},', lua_font_string, re.DOTALL)

    c_output_lines = []
    
    # Define the C array name and header, adjusting the internal size to 8 (8x8 font)
    # The inner array size should be 8 for an 8x8 font (8 bytes/char).
    c_output_lines.append(
        "// Converted from Lua font data with 90-degree counter-clockwise rotation."
    )
    c_output_lines.append(
        "#include <stdint.h>"
    )
    c_output_lines.append(
        "static const uint8_t font8x8[][8] = {"
    )

    for char_index, char_block_content in enumerate(char_blocks):
        # 2. Extract all numbers (0s and 1s) from the block content
        all_bits = [int(n) for n in re.findall(r'\d+', char_block_content)]
        
        # An 8x8 font has 64 bits (8 bytes) per character
        if len(all_bits) < 64:
            # Skip incomplete characters
            sys.stderr.write(f"Warning: Character {char_index} has only {len(all_bits)} bits (expected 64). Skipping.\n")
            continue
        
        # --- 90 Degree Counter-Clockwise Rotation Logic ---
        # 1. Convert the 64 flat bits (read row-by-row) into an 8x8 matrix (Row: 0-7, Column: 0-7).
        matrix = []
        for r in range(8):
            matrix.append(all_bits[r*8 : (r+1)*8])

        # 2. Generate the 8 new rows (bytes) from the rotated matrix.
        # 90 deg counter-clockwise rotation: 
        # New Row 'i' is generated from the Original Column 'i', 
        # with bits read *bottom-up* (from original row 7 down to row 0).
        hex_bytes = []
        for new_row_index in range(8):
            # The source column in the original matrix is the same as the new row index
            src_col_index = new_row_index
            
            # The new row bits are the bits from the source column, read bottom-up (r=7 down to r=0)
            # The `reversed(range(8))` ensures we read rows 7, 6, 5, ..., 0.
            new_row_bits = [matrix[r][src_col_index] for r in reversed(range(8))]

            hex_bytes.append(convert_binary_to_hex_byte(new_row_bits))
        
        # Format the line for C array
        line = f"    {{{', '.join(hex_bytes)}}}, // Char {char_index} (Rotated 90 deg CCW)"
        c_output_lines.append(line)

    c_output_lines.append("};")
    return "\n".join(c_output_lines)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        sys.stderr.write("Usage: python lua_font_converter.py <input_lua_file>\n")
        sys.exit(1)

    input_file_path = sys.argv[1]

    if not os.path.exists(input_file_path):
        sys.stderr.write(f"Error: Input file '{input_file_path}' not found.\n")
        sys.exit(1)

    try:
        with open(input_file_path, 'r') as f:
            lua_data = f.read()
    except Exception as e:
        sys.stderr.write(f"Error reading file: {e}\n")
        sys.exit(1)

    # Execute the conversion and print to standard output
    c_array_output = convert_lua_font_to_c_array(lua_data)
    print(c_array_output)