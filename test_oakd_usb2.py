#!/usr/bin/env python3
import depthai as dai

print("🔍 Test en mode USB 2.0...")

pipeline = dai.Pipeline()
cam = pipeline.create(dai.node.ColorCamera)
cam.setPreviewSize(256, 256)
xout = pipeline.create(dai.node.XLinkOut)
xout.setStreamName("rgb")
cam.preview.link(xout.input)

try:
    print("📷 Connexion en USB 2.0...")
    # FORCER USB 2.0
    device = dai.Device(pipeline, usb2Mode=True)
    print("✓ Device initialisé en USB 2.0 !")
    
    q = device.getOutputQueue("rgb", maxSize=1, blocking=True)
    frame = q.get()
    print(f"✓ Frame reçue : {frame.getCvFrame().shape}")
    
    device.close()
    print("✅ Test réussi en USB 2.0 !")
    
except Exception as e:
    print(f"❌ Erreur: {e}")
