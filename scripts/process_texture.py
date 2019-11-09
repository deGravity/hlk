#!/usr/bin/python

import imageio
import numpy as np
import os

def process_texture(name):
    png_file = f'../resources/{name}.png'
    key_file = f'../resources/{name}.txt'
    out_file = f'../src/{name}.h'
    
    if not os.path.exists(png_file):
        print(f'No such texture: {png_file}')
        return
    im = imageio.imread(png_file)

    (h,w,d) = im.shape

    with open(out_file, 'w') as f:
        write_header(f, name)

        if os.path.exists(key_file):
            glyph_set = read_key_file(key_file, w, h)
            for glyph in glyph_set.keys():
                glyph_data = glyph_set[glyph]
                write_mat(f, glyph, glyph_data)
        """ currently buggy - use readPNG for now
        write_uchar_mat_2(f, 'R', im[:,:,0])
        write_uchar_mat_2(f, 'G', im[:,:,1])
        write_uchar_mat_2(f, 'B', im[:,:,2])
        if im.shape[2] > 3:
            write_uchar_mat_2(f, 'A', im[:,:,3])
        """
        write_footer(f)

def write_header(f, name):
    f.write('#pragma once\n')
    f.write('\n')
    f.write('#include <Eigen/Core>\n')
    f.write('\n')
    f.write('namespace hlk {\n')
    f.write(f'namespace {name} {{\n')

def write_footer(f):
    f.write('}\n')
    f.write('}\n')
    f.write('\n')

def write_mat(f, name, mat):
    (h, w) = mat.shape
    f.write(f'static Eigen::MatrixXd {name} = [] {{\n')
    f.write(f'\tEigen::Matrix<double, {h}, {w}> tmp;\n')
    f.write(f'\t tmp <<')
    for r in range(0, mat.shape[0]):
        f.write('\n\t\t')
        for c in range(0, mat.shape[1]):
            val = mat[r][c]
            if c < mat.shape[1] - 1  or r < mat.shape[0] - 1:
                f.write(f'{val}, ')
            else:
                f.write(f'{val};\n')
    f.write('\treturn tmp;\n')
    f.write('}();\n\n')

def write_uchar_mat_2(f, name, mat):
    (h, w) = mat.shape
    SIZE = h*w
    f.write(f'static Eigen::Matrix<unsigned char, -1, -1> {name} = [] {{\n')
    f.write(f'\tunsigned char data[{SIZE}] = {{')
    for r in range(0, mat.shape[0]):
        f.write('\n\t\t')
        for c in range(0, mat.shape[1]):
            val = mat[r][c]
            if c < mat.shape[1] - 1  or r < mat.shape[0] - 1:
                f.write(f'{val},')
            else:
                f.write(f'{val}}};\n')
    f.write(f'\treturn Eigen::Map<Eigen::Matrix<unsigned char, -1, -1>>(data,{w},{h});\n')
    f.write('}();\n\n')
    

def write_uchar_mat(f, name, mat):
    (h, w) = mat.shape
    f.write(f'static Eigen::Matrix<unsigned char, -1, -1> {name} = [] {{\n')
    f.write(f'\tEigen::Matrix<unsigned char, {h}, {w}> tmp;\n')
    f.write(f'\t tmp <<')
    for r in range(0, mat.shape[0]):
        f.write('\n\t\t')
        for c in range(0, mat.shape[1]):
            val = mat[r][c]
            if c < mat.shape[1] - 1  or r < mat.shape[0] - 1:
                f.write(f'{val},')
            else:
                f.write(f'{val};\n')
    f.write('\treturn tmp;\n')
    f.write('}();\n\n')
    
def read_key_file(filename, w, h):
    with open(filename, 'r') as f:
        lines = list(x.strip() for x in f.readlines())
    
    glyphs = {}
    assert(len(lines) % 2 == 0)
    for i in range(0, int(len(lines) / 2)):
        name = lines[2*i]
        pos = list(float(x.strip()) for x in lines[2*i+1].split(','))
        glyphs[name] = pos
    
    glyph_mats = {}
    for name in glyphs.keys():
        pos = glyphs[name]
        a = (pos[0] / w, (h-(pos[1] + pos[3]))/h)
        d = (pos[2] / w, pos[3] / h)
        mat = np.array([[d[0], 0],[0, d[1]],[a[0], a[1]]])
        glyph_mats[name] = mat
    
    return glyph_mats

import sys
if __name__ == "__main__":
    if len(sys.argv) < 2:
        print('Usage: process_texture [textures...]')
    else:
        for name in sys.argv[1:]:
            process_texture(name)
