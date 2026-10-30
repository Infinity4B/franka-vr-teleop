#!/usr/bin/env python3
"""从 Noto Sans SC 可变字体裁出界面用到的字（vr-app/scripts/*.gd 里的所有字符 + ASCII）。

  pip install fonttools
  python3 make_font.py NotoSansSC[wght].ttf

字体来源：https://github.com/google/fonts/tree/main/ofl/notosanssc（SIL OFL 1.1）。
界面文字改动后重跑，否则新字会显示成方框。
"""
import glob
import os
import string
import sys

from fontTools import subset
from fontTools.ttLib import TTFont
from fontTools.varLib import instancer

HERE = os.path.dirname(os.path.abspath(__file__))
APP = os.path.join(HERE, '..', 'vr-app')
OUT = os.path.join(APP, 'assets', 'fonts', 'NotoSansSC-UI.ttf')


def main():
    src = sys.argv[1]
    chars = set(string.printable) | set('⌫°×—…（）：，。、％“”')
    for path in glob.glob(os.path.join(APP, 'scripts', '*.gd')):
        with open(path, encoding='utf-8') as f:
            chars |= set(f.read())
    chars = {c for c in chars if c.isprintable() or c == ' '}

    font = TTFont(src)
    if 'fvar' in font:
        font = instancer.instantiateVariableFont(font, {'wght': 500})
    opts = subset.Options()
    opts.layout_features = ['*']
    opts.name_IDs = ['*']
    opts.notdef_outline = True
    sub = subset.Subsetter(opts)
    sub.populate(text=''.join(sorted(chars)))
    sub.subset(font)
    os.makedirs(os.path.dirname(OUT), exist_ok=True)
    font.save(OUT)
    print(f'{len(chars)} 个字符 → {OUT}（{os.path.getsize(OUT) // 1024} KB）')


if __name__ == '__main__':
    main()
