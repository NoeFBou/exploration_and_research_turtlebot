#!/usr/bin/env python3
import depthai as dai

print("🔍 Test de la caméra OAK-D...")

# Liste des devices
devices = dai.Device.getAllAvailableDevices()
print(f"✓ Devices trouvés: {len(devices)}")

# Pipeline minimaliste (juste RGB)
pipeline = dai.Pipeline()

cam_rgb = pipeline.create(dai.node.ColorCamera)
cam_rgb.setPreviewSize(256, 256)
cam_rgb.setInterleaved(False)

xout_rgb = pipeline.create(dai.node.XLinkOut)
xout_rgb.setStreamName("rgb")
cam_rgb.preview.link(xout_rgb.input)

print("✓ Pipeline créé")

# Initialiser le device avec timeout plus long
try:
    print("📷 Connexion au device...")
    device = dai.Device(pipeline)
    print("✓ Device initialisé avec succès !")
    
    # Test de récupération d'une frame
    q = device.getOutputQueue("rgb", maxSize=4, blocking=False)
    print("✓ Queue créée")
    
    frame = q.get()
    print(f"✓ Frame reçue : {frame.getCvFrame().shape}")
    
    device.close()
    print("✅ Test réussi !")
    
except Exception as e:
    print(f"❌ Erreur: {e}")
    import traceback
    traceback.print_exc()
