from PIL import Image, ImageDraw
MAP = '/home/hamad/maps/majlis_v3.pgm'
BACKUP = '/home/hamad/maps/majlis_v3_BACKUP_pretable.pgm'
RES = 0.05
ORIGIN_X = -25.0
ORIGIN_Y = -25.0
# Restore from clean backup first, so we don't paint on top of the bad box
im = Image.open(BACKUP).copy()
W, H = im.size
print("Map image size:", W, "x", H)
# The 4 table corners (map meters). C3/C4 pulled ~18cm off the L/M wall
# to match the real 85cm gap (was sealing the channel at 62.8cm).
corners_m = [
    (3.500, 0.714),
    (4.050, 0.880),
    (3.811, 1.668),
    (3.273, 1.595),
]
# Expand polygon outward from its centroid by a small margin (meters)
MARGIN = 0.04
cx = sum(p[0] for p in corners_m) / 4.0
cy = sum(p[1] for p in corners_m) / 4.0
def expand(p):
    dx, dy = p[0]-cx, p[1]-cy
    import math
    d = math.hypot(dx, dy)
    if d == 0: return p
    return (p[0] + dx/d*MARGIN, p[1] + dy/d*MARGIN)
corners_m = [expand(p) for p in corners_m]
def to_px(x, y):
    col = (x - ORIGIN_X) / RES
    row = H - 1 - (y - ORIGIN_Y) / RES
    return (col, row)
poly_px = [to_px(x, y) for (x, y) in corners_m]
print("Polygon pixels:", [(round(c),round(r)) for c,r in poly_px])
draw = ImageDraw.Draw(im)
draw.polygon(poly_px, fill=0)  # 0 = black = occupied
print("Filled rotated table polygon (margin %.2fm)" % MARGIN)
im.save(MAP)
print("Saved", MAP)
