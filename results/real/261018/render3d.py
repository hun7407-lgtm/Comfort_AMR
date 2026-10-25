#!/usr/bin/env python3
"""Write a PLY and render the accumulated cloud from a few angles."""
import numpy as np, cv2, math, sys

SP="/tmp/claude-1000/-home-hun-Desktop/7b3174d9-ddc1-4af9-974d-05e20722647d/scratchpad"
SP="/tmp/claude-1000/-home-hun-Desktop/7b3174d9-ddc1-4af9-974d-05e20722947d/scratchpad"
OUT="/home/hun/Desktop/Comfort_AMR/results/real/261018"
A = np.load(f"{SP}/cloud3d.npz")["xyz"]
print("points", len(A))

# colour by height, turbo-ish
z = A[:,2]
t = np.clip((z - 0.3) / 2.2, 0, 1)
col = (cv2.applyColorMap((t*255).astype(np.uint8).reshape(-1,1), cv2.COLORMAP_TURBO)
       .reshape(-1,3))                      # BGR

# ---- PLY (binary, with colour) --------------------------------------------
ply = f"{OUT}/corridor_cloud.ply"
with open(ply, "wb") as f:
    f.write(b"ply\nformat binary_little_endian 1.0\n")
    f.write(f"element vertex {len(A)}\n".encode())
    f.write(b"property float x\nproperty float y\nproperty float z\n")
    f.write(b"property uchar red\nproperty uchar green\nproperty uchar blue\n")
    f.write(b"end_header\n")
    rec = np.empty(len(A), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),
                                  ('r','u1'),('g','u1'),('b','u1')])
    rec['x'],rec['y'],rec['z'] = A[:,0],A[:,1],A[:,2]
    rec['r'],rec['g'],rec['b'] = col[:,2],col[:,1],col[:,0]
    f.write(rec.tobytes())
import os; print("wrote", ply, "%.1f MB" % (os.path.getsize(ply)/1e6))

# ---- renderer --------------------------------------------------------------
W,H = 1600, 760
CENTER = A.mean(0)

def render(az, el, dist, title, zoom=1150.0):
    # zoom is a focal length in PIXELS. At 70 a 17 m corridor seen from 16 m
    # projected to 37 px and the whole cloud collapsed to a smudge.
    ca,sa = math.cos(az), math.sin(az)
    ce,se = math.cos(el), math.sin(el)
    P = A - CENTER
    # yaw then pitch
    xr = P[:,0]*ca + P[:,1]*sa
    yr = -P[:,0]*sa + P[:,1]*ca
    zr = P[:,2]
    yc = yr*ce - zr*se
    zc = yr*se + zr*ce
    depth = yc + dist
    good = depth > 0.2
    u = (W/2 + zoom*xr[good]/depth[good]*1.0).astype(int)
    v = (H/2 - zoom*zc[good]/depth[good]*1.0).astype(int)
    d = depth[good]; c = col[good]
    ok = (u>=0)&(u<W)&(v>=0)&(v<H)
    u,v,d,c = u[ok],v[ok],d[ok],c[ok]
    order = np.argsort(-d)              # far first, near overwrites
    u,v,c = u[order],v[order],c[order]
    img = np.full((H,W,3), 16, np.uint8)
    img[v,u] = c
    img = cv2.dilate(img, np.ones((2,2),np.uint8))
    cv2.putText(img,title,(16,34),cv2.FONT_HERSHEY_SIMPLEX,0.72,(255,255,255),2)
    cv2.putText(img,f'{len(A):,} points  -  colour = height 0.3-2.5 m  -  real AMR, 96 s drive',
                (16,60),cv2.FONT_HERSHEY_SIMPLEX,0.46,(190,190,190),1)
    return img

views = [("iso",   math.radians(-62), math.radians(24), 15.0, "3D point cloud  -  isometric"),
         ("side",  math.radians(-90), math.radians(3),  11.0, "3D point cloud  -  along the corridor"),
         ("top",   math.radians(-90), math.radians(86), 13.0, "3D point cloud  -  from above")]
for name, az, el, dist, title in views:
    cv2.imwrite(f"{OUT}/cloud3d_{name}.png", render(az,el,dist,title))
    print("wrote", f"cloud3d_{name}.png")

# ---- turntable gif ---------------------------------------------------------
frames=[]
for i in range(30):
    az = math.radians(-58) + i*2*math.pi/30
    f = render(az, math.radians(22), 15.0, "3D point cloud  -  real corridor")
    frames.append(cv2.cvtColor(cv2.resize(f,(W//2,H//2)), cv2.COLOR_BGR2RGB))
from PIL import Image
ims=[Image.fromarray(f).quantize(colors=128) for f in frames]
ims[0].save(f"{OUT}/cloud3d_turntable.gif", save_all=True, append_images=ims[1:],
            duration=110, loop=0, optimize=True)
print("wrote cloud3d_turntable.gif %.1f MB" % (os.path.getsize(f"{OUT}/cloud3d_turntable.gif")/1e6))
