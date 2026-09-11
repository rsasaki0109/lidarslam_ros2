#ifndef M6A10_V12_TEST_STUB_TRIGGER_H
#define M6A10_V12_TEST_STUB_TRIGGER_H

#include <string>

namespace std_srvs {

struct Trigger
{
  struct Request {};
  struct Response
  {
    bool success = false;
    std::string message;
  };
};

}  // namespace std_srvs

#endif  // M6A10_V12_TEST_STUB_TRIGGER_H
