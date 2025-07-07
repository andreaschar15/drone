import roboflow

rf = roboflow.Roboflow(api_key="")
model = rf.workspace().project("crossing-marker").version("1").model
prediction = model.download()
