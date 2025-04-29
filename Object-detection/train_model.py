from ultralytics import YOLO


model = YOLO('yolov8n.yaml')  

#train
model.train(
    data='path/to/mango_dataset/mango.yaml',  
    epochs=100,                     
    batch=4,            
    device=0,            
    name='mango_detector',  
    patience=30,
    workers=0,         
    optimizer='SGD',     
    lr0=0.002,           
    weight_decay=0.0005,  
    close_mosaic=10,     
    verbose=True
)


#validate
model = YOLO('runs/detect/mango_detector/weights/best.pt')


metrics = model.val(
    data='path/to/mango_dataset/mango.yaml',
    split='test',
    device=0  
)


print(metrics)

#test
results = model.predict(
    source='path/to/your/test_image_or_folder',
    save=True,
    conf=0.5,
    device=0   
)

#export
#model.export(format='torchscript')