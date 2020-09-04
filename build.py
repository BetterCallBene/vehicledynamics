import os
import shutil
import sys
from OMPython import OMCSessionZMQ

import unittest

class OMPythonError(Exception):
    
    def __init__(self, message):
        self.message = message
        super().__init__(self.message)

    def __str__(self):
        return f'{self.message}'


class ModelicaSystem(object):

    def __init__(self, fileName = None):
        self.__conn = None
        self.__fileName = fileName
        self.__isopen = False

    def __enter__(self):
        self.open()
        return self

    def __exit__(self, exc_type, exc_value, exc_traceback):
        self.close()

    def __requestApi(self, request):

        if not self.__isopen or self.__conn is None:
            raise OMPythonError('Error: No initialization was done.')
        if not self.__conn.sendExpression(request):
            errorCode = self.__conn.sendExpression("getErrorString()")
            
            if errorCode != '':
                self.__conn.sendExpression("quit()")
                raise OMPythonError(f'Error: {errorCode}')

    def __loadModelica(self):
        self.__requestApi('loadModel(Modelica)')

    def __loadFile(self, fileName):
        self.__requestApi('loadFile("{0}")'.format(self.__fileName))

    def checkModel(self, modelName):
        self.__requestApi('checkModel({0})'.format(modelName))

    def buildModel(self, modelName):
        self.__requestApi('buildModel({0})'.format(modelName))

    def simulate(self, modelName):
        self.__requestApi('simulate({0})'.format(modelName))

    def open(self):
        if self.__isopen:
            return
        self.__conn = OMCSessionZMQ()
        self.__isopen = True
        self.__loadModelica()
        self.__loadFile(self.__fileName)
        

    def close(self):
        self.__conn.sendExpression('quit()')
        self.__isopen = False

    @property
    def is_open(self):
        return self.__isopen

    
class TestModelicaSystem(unittest.TestCase):

    def setUp(self):
        self.modelName = "VehicleDynamics.Suspensions.Tests.MacPherson"
        workingDirectory = 'build'
        packageFolder = os.path.join('..', 'VehicleDynamics')
        packageFile = os.path.join(packageFolder, 'package.mo')
        self.packageFile = os.path.abspath(packageFile)
        if os.path.isdir(workingDirectory):
            shutil.rmtree(workingDirectory) 
        os.mkdir(workingDirectory)
        os.chdir(workingDirectory)

    def test_initializition(self):
        mc = ModelicaSystem(self.packageFile)
        self.assertEqual(mc.is_open, False)
        mc.open()
        self.assertEqual(mc.is_open, True)
        mc.close()
        self.assertEqual(mc.is_open, False)

    def test_connext_manager(self):
        mc = ModelicaSystem(self.packageFile)
        with mc:
            self.assertEqual(mc.is_open, True)
        self.assertEqual(mc.is_open, False)

    def test_build(self):
        with ModelicaSystem(self.packageFile) as mc:
            mc.buildModel(self.modelName)      
            
def build():
    modelName = "VehicleDynamics.Suspensions.Tests.MacPherson"

    workingDirectory = 'build'
    packageFolder = os.path.join('..', 'VehicleDynamics')
    packageFile = os.path.join(packageFolder, 'package.mo')
    
    if os.path.isdir(workingDirectory):
        shutil.rmtree(workingDirectory) 
    os.mkdir(workingDirectory)
    os.chdir(workingDirectory)

    try:
        with ModelicaSystem(fileName=packageFile) as mc:
            mc.checkModel(modelName)
            mc.buildModel(modelName)
            mc.simulate(modelName)
    except OMPythonError as err:
        print(err.args[0])
        sys.exit(1)

    
# if __name__ == "__main__":
#     build()