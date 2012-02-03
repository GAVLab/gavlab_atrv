#include "atrv/atrv.h"

using namespace atrv;

void
encoderCallback(size_t motor_index,
                const mdc2250::queries::QueryType &query_type,
                std::vector<long> &telemetry)
{
  if (motor_index == 1) {
    std::cout << "Front ";
  } else if (motor_index == 2) {
    std::cout << "Rear ";
  } else {
    std::cout << "Unknown ";
  }
  std::cout << "encoders: " << telemetry[0] << ":" << telemetry[1];
  std::cout << std::endl;
}

void
generalCallback(size_t motor_index,
                const mdc2250::queries::QueryType &query_type,
                std::vector<long> &telemetry)
{
  if (motor_index == 1) {
    std::cout << "Front ";
  } else if (motor_index == 2) {
    std::cout << "Rear ";
  } else {
    std::cout << "Unknown ";
  }
  std::cout << mdc2250::response_type_to_string(query_type) << ": ";
  std::vector<long>::iterator it = telemetry.begin();
  while (it != telemetry.end()) {
    std::cout << (*it);
    ++it;
    if (it != telemetry.end()) {
      std::cout << ":";
    }
  }
  std::cout << std::endl;
}

int run() {
  try {
    ATRV my_atrv;
    my_atrv.connect("/dev/tty.USA49Wfd122P2.2",
                    "/dev/tty.USA49Wfd122P1.1", 3000, false);

    // Setup Telemetry
    using namespace mdc2250::queries;
    my_atrv.setTelemetryCallback(encoderCallback, encoder_count_absolute);
    my_atrv.setTelemetryCallback(generalCallback, any_query);

    std::cout << "Moving..." << std::endl;
    my_atrv.move(1.0, 0.0);
    boost::this_thread::sleep(boost::posix_time::milliseconds(5000));
  } catch (std::exception &e) {
    std::cerr << "Unhandled exception: " << e.what() << std::endl;
    throw(e);
  }

  return 0;
}

int main(void) {
  return run();
}
