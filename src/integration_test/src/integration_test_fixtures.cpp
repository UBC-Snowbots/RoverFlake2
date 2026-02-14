// This class is meant to be a helper class for integration tests.
#include <string>
#include <unordered_map>
#include <cstdint>

// Work in progress...

// A single test
class TestRunner {
public:
    TestRunner(std::string name) {};

    void assert(bool assertion) 
    {

    }

    void assert(float val1, float val2, float thresh = 0.0000001)
    {

    }


private:
    std::string name;
};

class IntegrationTestHelper {
public:
    IntegrationTestHelper() : next_test_index(0), current_test_index(0) {};

    TestRunner& add_test(std::string name);

private:
    std::unordered_map<uint32_t, TestRunner> tests;
    uint32_t next_test_index;
    uint32_t current_test_index;
};

TestRunner& IntegrationTestHelper::add_test(std::string name)
{
    current_test_index = next_test_index;
    tests.emplace(current_test_index, TestRunner(name));
    next_test_index++;
    return tests[current_test_index];
}





// Example usage:

// auto new_test = test.add_test("My new test");