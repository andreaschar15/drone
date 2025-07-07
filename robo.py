import roboflow

rf = roboflow.Roboflow(api_key="ExTS2OjWNK8o7W4oZcqs")
model = rf.workspace().project("crossing-marker").version("1").model
prediction = model.download()
