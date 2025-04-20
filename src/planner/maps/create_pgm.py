#!/usr/bin/env python3

def create_empty_pgm(filename, width, height):
    with open(filename, 'wb') as f:
        # Header
        f.write(b'P5\n')               # PGM binary format
        f.write(f'{width} {height}\n'.encode())  # Width and height
        f.write(b'255\n')              # Maximum gray value
        
        # Write pixel data (all 255 for white/empty)
        # Writing all at once is much faster than byte-by-byte
        f.write(b'\xff' * (width * height))

# Create a 1000x1000 empty PGM file
create_empty_pgm('empty_map.pgm', 1000, 1000)