import sys
import os

input_file = sys.argv[1]
output_file = sys.argv[2]

with open(input_file, 'r') as f_in, open(output_file, 'w') as f_out:
    for line in f_in:
        f_out.write(line)
        if line.startswith('f '):
            parts = line.strip().split()
            # parts[0] is 'f'
            # parts[1:] are the vertices
            reversed_verts = parts[1:][::-1]
            f_out.write('f ' + ' '.join(reversed_verts) + '\n')

print(f"Successfully processed {input_file} -> {output_file}")
