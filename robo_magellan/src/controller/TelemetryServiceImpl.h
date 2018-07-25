#pragma once
#include <iostream>
#include <memory>
#include <string>
#include <grpcpp/grpcpp.h>
#include <telemetry_service.grpc.pb.h>

#include "controller/Attitude.h"
#include "util/CircularBuffer.h"

using grpc::Server;
using grpc::ServerWriter;
using grpc::ServerBuilder;
using grpc::ServerContext;
using grpc::Status;
using telemetry::HitEvent;
using telemetry::EventResponse;
using telemetry::AttitudeQResponse;
using telemetry::AttitudeQRequest;
using telemetry::TelemetryService;

using telemetry::Waypoints;
using telemetry::HeadingEvent;
using telemetry::NoContent;

class TelemetryServiceImpl final : public TelemetryService::Service {
private:
	CircularBuffer<Attitude> * const attitudeBuffer;
public:	
	TelemetryServiceImpl(CircularBuffer<Attitude> *buffer) : attitudeBuffer(buffer) {};
	Status GetAttitudeQ(ServerContext* context, const AttitudeQRequest* request, AttitudeQResponse* reply) override;
	Status GetAttitudeStream(ServerContext* context, const AttitudeQRequest* request, ServerWriter< ::telemetry::AttitudeQResponse>* writer) override;
	Status hitEvent(ServerContext* context, const HitEvent* request, EventResponse* response) override;
    Status headingEvent(ServerContext* context, const HeadingEvent* request, EventResponse* response);
    // Navigation
    Status setTarget(ServerContext* context, const ::telemetry::Waypoint* request, ::telemetry::NoContent* response);
    Status setWaypoints(ServerContext* context, const Waypoints* request, NoContent* response);

	static void RunServer(CircularBuffer<Attitude> * const buffer, bool wait);
};
