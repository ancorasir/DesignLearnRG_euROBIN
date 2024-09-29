#include "DataRelay.h"

int main(int argc, char *argv[])
{
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    DataRelayApp app;
    app.run();
    google::protobuf::ShutdownProtobufLibrary();

    return 0;
}
