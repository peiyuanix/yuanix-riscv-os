#!/usr/bin/env python3
# Figma to baremetal RISCV: by Brent Hartshorn
# install: sudo apt-get install gcc-riscv64-unknown-elf qemu-system-riscv64

import os, sys, subprocess, json, string
from PIL import Image, ImageFont, ImageDraw

TEST = 'https://www.figma.com/design/J9oh3PpkkivuV5xp5p5go9/Untitled?node-id=0-1&t=19PvBbLLyJstPZBs-1'
TOKEN = 'figd_pt53oO8QXO7v2YwYF4CNxo4sHMznjb9PJ6P4UpXq'


## https://github.com/Amatobahn/FigmaPy
if not os.path.isdir('./FigmaPy'):
	cmd = ['git', 'clone', '--depth', '1', 'https://github.com/Amatobahn/FigmaPy.git']
	subprocess.check_call(cmd)

assert os.path.isdir('./FigmaPy')
sys.path.append('./FigmaPy')
import figmapy
print(figmapy)

LIB_DRAW = '''
void draw_hline(int x, int y, int w, int color){
	for (int i=x; i<w; i++) putpixel(i,y,color);
}
void draw_fill(int x, int y, int w, int h, int color){
	for (int yi=0; yi<h; yi++){
		draw_hline(x, y+yi, w, color);
	}
}
'''

def gen_font( size=10):
	ascii = {}
	out = []
	for c in string.ascii_letters + string.digits + string.punctuation:
		img = Image.new(('RGB'), (size,size))
		draw = ImageDraw.Draw(img)
		draw.text((0,0), c)
		#img = img.convert('P')
		ascii[c] = o = ['void draw_char_%s(int x, int y, int color){	//%s' % (ord(c),c) ]
		#for pidx, pixel in enumerate(img.getdata()):
		for y in range(size):
			for x in range(size):
				r,g,b = img.getpixel((x,y))
				if r:
					o.append('	putpixel(%s+x,%s+y,color);' %(x,y))

		o.append('}')
		out += o
	return '\n'.join(out)

LIB_FONT = gen_font()


vga_pal = [0x00, 0x00, 0x00,0x00, 0x00, 0x55,0x00, 0x00, 0xAA,0x00, 0x00, 0xFF,0x00, 0x55, 0x00,0x00, 0x55, 0x55,0x00, 0x55, 0xAA,0x00, 0x55, 0xFF,0x00, 0xAA, 0x00,0x00, 0xAA, 0x55,0x00, 0xAA, 0xAA,0x00, 0xAA, 0xFF,0x00, 0xFF, 0x00,0x00, 0xFF, 0x55,0x00, 0xFF, 0xAA,0x00, 0xFF, 0xFF,0x55, 0x00, 0x00,0x55, 0x00, 0x55,0x55, 0x00, 0xAA,0x55, 0x00, 0xFF,0x55, 0x55, 0x00,0x55, 0x55, 0x55,0x55, 0x55, 0xAA,0x55, 0x55, 0xFF,0x55, 0xAA, 0x00,0x55, 0xAA, 0x55,0x55, 0xAA, 0xAA,0x55, 0xAA, 0xFF,0x55, 0xFF, 0x00,0x55, 0xFF, 0x55,0x55, 0xFF, 0xAA,0x55, 0xFF, 0xFF,0xAA, 0x00, 0x00,0xAA, 0x00, 0x55,0xAA, 0x00, 0xAA,0xAA, 0x00, 0xFF,0xAA, 0x55, 0x00,0xAA, 0x55, 0x55,0xAA, 0x55, 0xAA,0xAA, 0x55, 0xFF,0xAA, 0xAA, 0x00,0xAA, 0xAA, 0x55,0xAA, 0xAA, 0xAA,0xAA, 0xAA, 0xFF,0xAA, 0xFF, 0x00,0xAA, 0xFF, 0x55,0xAA, 0xFF, 0xAA,0xAA, 0xFF, 0xFF,0xFF, 0x00, 0x00,0xFF, 0x00, 0x55,0xFF, 0x00, 0xAA,0xFF, 0x00, 0xFF,0xFF, 0x55, 0x00,0xFF, 0x55, 0x55,0xFF, 0x55, 0xAA,0xFF, 0x55, 0xFF,0xFF, 0xAA, 0x00,0xFF, 0xAA, 0x55,0xFF, 0xAA, 0xAA,0xFF, 0xAA, 0xFF,0xFF, 0xFF, 0x00,0xFF, 0xFF, 0x55,0xFF, 0xFF, 0xAA,0xFF, 0xFF, 0xFF,0x00, 0x00, 0x00,0x00, 0x00, 0x55,0x00, 0x00, 0xAA,0x00, 0x00, 0xFF,0x00, 0x55, 0x00,0x00, 0x55, 0x55,0x00, 0x55, 0xAA,0x00, 0x55, 0xFF,0x00, 0xAA, 0x00,0x00, 0xAA, 0x55,0x00, 0xAA, 0xAA,0x00, 0xAA, 0xFF,0x00, 0xFF, 0x00,0x00, 0xFF, 0x55,0x00, 0xFF, 0xAA,0x00, 0xFF, 0xFF,0x55, 0x00, 0x00,0x55, 0x00, 0x55,0x55, 0x00, 0xAA,0x55, 0x00, 0xFF,0x55, 0x55, 0x00,0x55, 0x55, 0x55,0x55, 0x55, 0xAA,0x55, 0x55, 0xFF,0x55, 0xAA, 0x00,0x55, 0xAA, 0x55,0x55, 0xAA, 0xAA,0x55, 0xAA, 0xFF,0x55, 0xFF, 0x00,0x55, 0xFF, 0x55,0x55, 0xFF, 0xAA,0x55, 0xFF, 0xFF,0xAA, 0x00, 0x00,0xAA, 0x00, 0x55,0xAA, 0x00, 0xAA,0xAA, 0x00, 0xFF,0xAA, 0x55, 0x00,0xAA, 0x55, 0x55,0xAA, 0x55, 0xAA,0xAA, 0x55, 0xFF,0xAA, 0xAA, 0x00,0xAA, 0xAA, 0x55,0xAA, 0xAA, 0xAA,0xAA, 0xAA, 0xFF,0xAA, 0xFF, 0x00,0xAA, 0xFF, 0x55,0xAA, 0xFF, 0xAA,0xAA, 0xFF, 0xFF,0xFF, 0x00, 0x00,0xFF, 0x00, 0x55,0xFF, 0x00, 0xAA,0xFF, 0x00, 0xFF,0xFF, 0x55, 0x00,0xFF, 0x55, 0x55,0xFF, 0x55, 0xAA,0xFF, 0x55, 0xFF,0xFF, 0xAA, 0x00,0xFF, 0xAA, 0x55,0xFF, 0xAA, 0xAA,0xFF, 0xAA, 0xFF,0xFF, 0xFF, 0x00,0xFF, 0xFF, 0x55,0xFF, 0xFF, 0xAA,0xFF, 0xFF, 0xFF,0x00, 0x00, 0x00,0x00, 0x00, 0x55,0x00, 0x00, 0xAA,0x00, 0x00, 0xFF,0x00, 0x55, 0x00,0x00, 0x55, 0x55,0x00, 0x55, 0xAA,0x00, 0x55, 0xFF,0x00, 0xAA, 0x00,0x00, 0xAA, 0x55,0x00, 0xAA, 0xAA,0x00, 0xAA, 0xFF,0x00, 0xFF, 0x00,0x00, 0xFF, 0x55,0x00, 0xFF, 0xAA,0x00, 0xFF, 0xFF,0x55, 0x00, 0x00,0x55, 0x00, 0x55,0x55, 0x00, 0xAA,0x55, 0x00, 0xFF,0x55, 0x55, 0x00,0x55, 0x55, 0x55,0x55, 0x55, 0xAA,0x55, 0x55, 0xFF,0x55, 0xAA, 0x00,0x55, 0xAA, 0x55,0x55, 0xAA, 0xAA,0x55, 0xAA, 0xFF,0x55, 0xFF, 0x00,0x55, 0xFF, 0x55,0x55, 0xFF, 0xAA,0x55, 0xFF, 0xFF,0xAA, 0x00, 0x00,0xAA, 0x00, 0x55,0xAA, 0x00, 0xAA,0xAA, 0x00, 0xFF,0xAA, 0x55, 0x00,0xAA, 0x55, 0x55,0xAA, 0x55, 0xAA,0xAA, 0x55, 0xFF,0xAA, 0xAA, 0x00,0xAA, 0xAA, 0x55,0xAA, 0xAA, 0xAA,0xAA, 0xAA, 0xFF,0xAA, 0xFF, 0x00,0xAA, 0xFF, 0x55,0xAA, 0xFF, 0xAA,0xAA, 0xFF, 0xFF,0xFF, 0x00, 0x00,0xFF, 0x00, 0x55,0xFF, 0x00, 0xAA,0xFF, 0x00, 0xFF,0xFF, 0x55, 0x00,0xFF, 0x55, 0x55,0xFF, 0x55, 0xAA,0xFF, 0x55, 0xFF,0xFF, 0xAA, 0x00,0xFF, 0xAA, 0x55,0xFF, 0xAA, 0xAA,0xFF, 0xAA, 0xFF,0xFF, 0xFF, 0x00,0xFF, 0xFF, 0x55,0xFF, 0xFF, 0xAA,0xFF, 0xFF, 0xFF,0x00, 0x00, 0x00,0x00, 0x00, 0x55,0x00, 0x00, 0xAA,0x00, 0x00, 0xFF,0x00, 0x55, 0x00,0x00, 0x55, 0x55,0x00, 0x55, 0xAA,0x00, 0x55, 0xFF,0x00, 0xAA, 0x00,0x00, 0xAA, 0x55,0x00, 0xAA, 0xAA,0x00, 0xAA, 0xFF,0x00, 0xFF, 0x00,0x00, 0xFF, 0x55,0x00, 0xFF, 0xAA,0x00, 0xFF, 0xFF,0x55, 0x00, 0x00,0x55, 0x00, 0x55,0x55, 0x00, 0xAA,0x55, 0x00, 0xFF,0x55, 0x55, 0x00,0x55, 0x55, 0x55,0x55, 0x55, 0xAA,0x55, 0x55, 0xFF,0x55, 0xAA, 0x00,0x55, 0xAA, 0x55,0x55, 0xAA, 0xAA,0x55, 0xAA, 0xFF,0x55, 0xFF, 0x00,0x55, 0xFF, 0x55,0x55, 0xFF, 0xAA,0x55, 0xFF, 0xFF,0xAA, 0x00, 0x00,0xAA, 0x00, 0x55,0xAA, 0x00, 0xAA,0xAA, 0x00, 0xFF,0xAA, 0x55, 0x00,0xAA, 0x55, 0x55,0xAA, 0x55, 0xAA,0xAA, 0x55, 0xFF,0xAA, 0xAA, 0x00,0xAA, 0xAA, 0x55,0xAA, 0xAA, 0xAA,0xAA, 0xAA, 0xFF,0xAA, 0xFF, 0x00,0xAA, 0xFF, 0x55,0xAA, 0xFF, 0xAA,0xAA, 0xFF, 0xFF,0xFF, 0x00, 0x00,0xFF, 0x00, 0x55,0xFF, 0x00, 0xAA,0xFF, 0x00, 0xFF,0xFF, 0x55, 0x00,0xFF, 0x55, 0x55,0xFF, 0x55, 0xAA,0xFF, 0x55, 0xFF,0xFF, 0xAA, 0x00,0xFF, 0xAA, 0x55,0xFF, 0xAA, 0xAA,0xFF, 0xAA, 0xFF,0xFF, 0xFF, 0x00,0xFF, 0xFF, 0x55,0xFF, 0xFF, 0xAA,0xFF, 0xFF, 0xFF]
def to_vga_color( clr ):
	r = int(clr['r'] * 255)
	g = int(clr['g'] * 255)
	b = int(clr['b'] * 255)
	score = None
	color = None
	idx = 0
	for i in range(0, len(vga_pal), 3):
		vr = vga_pal[i]
		vg = vga_pal[i]
		vb = vga_pal[i]
		diff = abs(r-vr) + abs(g-vg) + abs(b-vb)
		if color is None or diff < score:
			score = diff
			color = idx
		idx += 1
	return color

def id2c(id):
	return id.replace(':', '_')

def meta_to_metal( meta ):
	c = [LIB_DRAW]
	print(meta)

	c += [
		'void redraw_background(){',
		'	draw_fill(0,0, 320, 240, %s);' % to_vga_color(meta['bgcolor']),
		'}',
	]

	funcs = ['redraw_background']

	for elt in meta['frames']:
		x = int(elt['x'])
		y = int(elt['y'])
		w = int(elt['w'])
		h = int(elt['h'])

		funcs.append('redraw_%s' % id2c(elt['id']))

		c += [
			'void redraw_%s(){' % id2c(elt['id']),
		]
		if elt['type']=='TEXT':
			c.append('//printf("%s")' % elt['text'])
			for aidx, a in enumerate(elt['text']):
				cx = aidx * 10
				cx += x
				c.append('	draw_char_%s(%s,%s,100);' % (ord(a), cx, y) )
		else:
			if elt['fill']:
				color = to_vga_color(elt['fill'])
				c.append('	draw_fill(%s,%s, %s,%s, %s);' % (x,y, w,h, color))

			if elt['stroke']:
				color = to_vga_color(elt['stroke'])
				c.append('	draw_hline(%s,%s, %s, %s);' % (x,y, w, color))
				c.append('	draw_hline(%s,%s, %s, %s);' % (x,y+h, w, color))

		c.append('}')

	c.append('void main(){')
	for f in funcs:
		c.append( f + '();' )
	c.append('}')

	return '\n'.join(c)

def figma_to_meta(file_key):
	frames = []
	pngs = []
	ids = []
	meta = {'frames':frames, 'pngs':pngs, 'ids':ids}

	if figmapy is None:
		return ''
	if file_key.startswith('https://www.figma.com/file/'):
		file_key = file_key[len('https://www.figma.com/file/') : ]
	elif file_key.startswith('https://www.figma.com/design/'):
		file_key = file_key[len('https://www.figma.com/design/') : ]
	if '/' in file_key:
		file_key = file_key.split('/')[0]

	print(file_key)
	fig = figmapy.FigmaPy(token=TOKEN)
	print(fig)

	file = fig.get_file(key=file_key)
	if not file: raise RuntimeError('invalid file key: %s' % file_key)
	print(file)
	print([x.name for x in file.document.children])


	page1 = file.document.children[0]
	meta['bgcolor'] = page1.backgroundColor
	nodes = {}
	css = {}
	minx = None
	miny = None
	for n in page1.children:
		x = n.absoluteRenderBounds['x']
		y = n.absoluteRenderBounds['y']
		if minx is None or x < minx:
			minx = x
		if miny is None or y < miny:
			miny = y

	print('minx:', minx)
	print('miny:', miny)


	for n in page1.children:
		ids.append(n.id)
		#print(dir(n))
		print(n.name)
		print(n.type)
		print('isFixed', n.isFixed)
		print('layoutAlign', n.layoutAlign)
		print('size', n.size)
		print('strokeWeight', n.strokeWeight)
		print('styles', n.styles)
		print('fills', n.fills)
		print('strokes', n.strokes)

		x = n.absoluteBoundingBox.x
		y = n.absoluteBoundingBox.y
		w = n.absoluteBoundingBox.width
		h = n.absoluteBoundingBox.height
		print('bounding box: %s %s %s %s'  %(x,y,w,h))
		x = n.absoluteRenderBounds['x']
		y = n.absoluteRenderBounds['y']
		w = n.absoluteRenderBounds['width']
		h = n.absoluteRenderBounds['height']
		print('render: %s %s %s %s'  %(x,y,w,h))

		x, y = (x+abs(minx), y+abs(miny))
		print('abs x y:', x,y)

		elt = {
			'type':n.type,
			'id': n.id,
			'strokeWeight':n.strokeWeight,
			'x': x, 'y':y,
			'w': w, 'h':h,
			'stroke':None,
			'fill'  :None,
		}
		frames.append(elt)

		if n.fills:
			for f in n.fills:
				elt['fill'] = f.color  ## this is a dict

		if n.strokes:
			for s in n.strokes:
				elt['stroke'] = s.color

		if n.type=="TEXT":
			elt['text'] = n.characters

	return meta

if __name__=='__main__':
	tmp = '/tmp/figma-cache.json'
	if '--offline' in sys.argv:
		m = json.loads( open(tmp, 'rb').read() )
	else:
		url = TEST
		for arg in sys.argv:
			if arg.startswith('https://www.figma.com/'):
				url = arg
			elif arg.startswith('figd_'):
				TOKEN = arg
		m = figma_to_meta(url)
		open(tmp, 'wb').write(json.dumps(m).encode('utf-8'))

	c = meta_to_metal(m)
	print(c)
