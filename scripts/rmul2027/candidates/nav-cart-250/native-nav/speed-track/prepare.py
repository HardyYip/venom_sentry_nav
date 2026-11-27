"""Build a dedicated open test lane and reference map for the straight-line speed check."""
from pathlib import Path
import sys
import json
import math
from xml.etree import ElementTree as ET

from PIL import Image, ImageDraw
import yaml

out = Path(sys.argv[1])
world = ET.parse(out / 'world.sdf')
scene = world.getroot().find('world')
for model in scene.findall('model'):
    if model.get('name') not in ('cart', 'box0'):
        scene.remove(model)
world.write(out / 'world.sdf', encoding='utf-8', xml_declaration=True)
goal = json.loads((out / 'case.json').read_text())['goal']
width = max(800, math.ceil((goal[0] + 11) / .05))
height = max(240, math.ceil((goal[1] + 12) / .05))
image = Image.new('L', (width, height), 254)
ImageDraw.Draw(image).rectangle((0, 0, width - 1, height - 1), outline=0, width=3)
image.save(out / 'lane.pgm')
metadata = {'image': 'lane.pgm', 'resolution': 0.05, 'origin': [-5.0, -6.0, 0.0],
            'negate': 0, 'occupied_thresh': 0.65, 'free_thresh': 0.196}
(out / 'lane.yaml').write_text(yaml.safe_dump(metadata))
