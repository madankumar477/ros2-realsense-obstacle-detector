#!/usr/bin/env python3
"""Render the exact data flow documented by the standalone package."""
from pathlib import Path
from PIL import Image, ImageDraw, ImageFont

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / 'assets' / 'system_architecture.png'
W, H = 1600, 620
im = Image.new('RGB', (W, H), (12, 20, 34))
d = ImageDraw.Draw(im)
font_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf'
bold_path = '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf'
def f(size, bold=False): return ImageFont.truetype(bold_path if bold else font_path, size)

d.text((55, 35), 'REALTIME DEPTH-BASED OBSTACLE DETECTION', font=f(34, True), fill=(238, 246, 255))
d.text((57, 88), 'ROS 2 Humble • Intel RealSense D435i • standalone detector node', font=f(21), fill=(141, 181, 224))

def panel(xy, fill=(25, 38, 59), outline=(76, 131, 194), radius=22, width=3):
    d.rounded_rectangle(xy, radius=radius, fill=fill, outline=outline, width=width)
def arrow(x1,y1,x2,y2,color=(125,186,255),width=5):
    d.line((x1,y1,x2,y2), fill=color, width=width)
    import math
    a=math.atan2(y2-y1,x2-x1); L=18; spread=0.55
    pts=[(x2,y2),(x2-L*math.cos(a-spread),y2-L*math.sin(a-spread)),(x2-L*math.cos(a+spread),y2-L*math.sin(a+spread))]
    d.polygon(pts, fill=color)

# Input
panel((55,205,405,445), fill=(23,51,73), outline=(62,177,220))
d.text((86,230), 'CAMERA INPUT', font=f(24,True), fill=(106,215,255))
d.text((86,292), 'Intel RealSense', font=f(26,True), fill=(245,248,255))
d.text((86,330), 'D435i depth stream', font=f(22), fill=(210,226,246))
d.text((86,370), 'Depth image + CameraInfo', font=f(18), fill=(155,189,224))
# Detector
panel((525,175,1080,475), fill=(35,35,75), outline=(153,113,255))
d.text((565,199), 'realsense_obstacle_detector', font=f(26,True), fill=(205,185,255))
d.text((568,252), '1  Decode depth (16UC1 / 32FC1)', font=f(21), fill=(237,241,255))
d.text((568,295), '2  Estimate floor plane per image row', font=f(21), fill=(237,241,255))
d.text((568,338), '3  Apply range + height + corridor filters', font=f(21), fill=(237,241,255))
d.text((568,381), '4  Fail-safe latch + heartbeat', font=f(21), fill=(237,241,255))
# Input arrow and labels
arrow(405,320,515,320)
d.text((406,265), 'Image + intrinsics', font=f(16), fill=(160,190,225))
# Outputs three cards
outs=[(1200,165,1540,270,'/realsense/obstacle_detected','std_msgs/Bool'),
      (1200,300,1540,405,'/realsense/min_obstacle_distance','std_msgs/Float32'),
      (1200,435,1540,540,'/realsense/obstacle_alert','std_msgs/String')]
for x1,y1,x2,y2,t,typ in outs:
    panel((x1,y1,x2,y2), fill=(23,49,48) if 'detected' in t else (28,41,61), outline=(67,194,151) if 'detected' in t else (80,143,211), radius=16, width=2)
    d.text((x1+16,y1+22),t,font=f(16,True),fill=(233,246,255))
    d.text((x1+16,y1+61),typ,font=f(17),fill=(164,199,224))
arrow(1080,320,1188,320)
# Branching routes
for yy in [217,352,487]:
    d.line((1140,320,1140,yy), fill=(125,186,255), width=3)
    arrow(1140,yy,1190,yy,width=3)
# Footer
d.text((58,565), 'Forward-motion policy is implemented by the downstream robot safety controller; this package publishes perception state only.', font=f(18), fill=(169,191,220))
im.save(OUT, optimize=True)
print(OUT)
