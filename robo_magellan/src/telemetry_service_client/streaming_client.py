from __future__ import print_function
from time import sleep

import grpc

import telemetry_service_pb2
import telemetry_service_pb2_grpc


def run():
    channel = grpc.insecure_channel('localhost:50051')
    stub = telemetry_service_pb2_grpc.TelemetryServiceStub(channel)
    responses = stub.GetAttitudeStream(telemetry_service_pb2.AttitudeQRequest())
    while 1==1:
       for response in responses:
          print('Received : %.2f' % response.Qc)
       sleep(1)
    # response = stub.SayHelloAgain(helloworld_pb2.HelloRequest(name='you'))
    # print("Greeter client received (again): " + response.message)


if __name__ == '__main__':
    run()

