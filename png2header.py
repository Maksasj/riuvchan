import sys
import os
from PIL import Image

def convert_image_to_c_header(input_file, output_file=None):
    # 1. Load the image
    try:
        im = Image.open(input_file)
    except IOError:
        print(f"Error: Could not open file '{input_file}'")
        sys.exit(1)

    # 2. Process Image (Resize to 128x64 and convert to 1-bit monochrome)
    # 128 pixels wide, 64 pixels high
    width, height = 128, 64
    
    # Resize with antialiasing to fit the OLED dimensions
    im = im.resize((width, height), Image.Resampling.LANCZOS)
    
    # Convert to 1-bit color (Black/White). 
    # By default, dithering is enabled. You can add .convert('1', dither=Image.NONE) for sharp edges.
    im = im.convert('1') 
    
    pixels = im.load()
    
    # 3. Generate the Buffer (SSD1306 "Page" format)
    # The display is divided into 8 pages (rows) of 8 pixels high.
    # We iterate page by page, then column by column.
    
    buffer = []
    
    for page in range(8): # 8 pages (0-7)
        for x in range(width): # 128 columns (0-127)
            byte = 0
            for bit in range(8): # 8 bits per byte (vertical pixels)
                y = page * 8 + bit
                # Check pixel color. 
                # In '1' mode, 255 is White (ON), 0 is Black (OFF).
                # Note: Depending on your specific OLED, you might need to invert this logic.
                if pixels[x, y] != 0: 
                    byte |= (1 << bit)
            buffer.append(byte)

    # 4. Generate Output String (C Header Format)
    filename = os.path.basename(input_file)
    var_name = os.path.splitext(filename)[0].replace(" ", "_").replace("-", "_").lower() + "_img"
    
    header_content = f"// Generated from {filename}\n"
    header_content += f"// Resolution: {width}x{height}\n"
    header_content += f"#ifndef {var_name.upper()}_H\n"
    header_content += f"#define {var_name.upper()}_H\n\n"
    header_content += f"#include <stdint.h>\n\n"
    header_content += f"static const uint8_t {var_name}[1024] = {{\n"
    
    for i, byte in enumerate(buffer):
        if i % 16 == 0:
            header_content += "    "
        header_content += f"0x{byte:02X}, "
        if (i + 1) % 16 == 0:
            header_content += "\n"
            
    header_content += "};\n\n"
    header_content += f"#endif // {var_name.upper()}_H\n"

    # 5. Write to file or stdout
    if output_file:
        with open(output_file, "w") as f:
            f.write(header_content)
        print(f"Success! Header file written to: {output_file}")
    else:
        print(header_content)

if __name__ == "__main__":
    if len(sys.argv) < 2:
        print("Usage: python png2header.py <input_image.png> [output_file.h]")
    else:
        input_img = sys.argv[1]
        output_h = sys.argv[2] if len(sys.argv) > 2 else input_img.rsplit('.', 1)[0] + ".h"
        convert_image_to_c_header(input_img, output_h)