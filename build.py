from OMPython import OMCSessionZMQ
from OMPython import ModelicaSystem
omc = OMCSessionZMQ()

ms = ModelicaSystem(fileName="/home/bettercallbene/workspace/vehicledynamics/VehicleDynamics/package.mo", modelName="VehicleDynamics.Suspensions.Tests.MacPherson")
omc.sendExpression('loadModel(Modelica)')
omc.sendExpression('getVersion()')
omc.sendExpression('loadFile("/home/bettercallbene/workspace/vehicledynamics/VehicleDynamics/package.mo")')

result = omc.sendExpression('simulate(VehicleDynamics.Suspensions.Tests.MacPherson)')
print(result)
