extern "C" {
#include "../util.h"
}

#include "CppUTest/CommandLineTestRunner.h"
#include "CppUTest/TestHarness.h"
#include "CppUTestExt/MockSupport.h"

TEST_GROUP(Util)
{
	void setup() {}
	void teardown() {
		mock().clear();
	}
};

TEST(Util, EmptyTest)
{
	char str[10];
	Itoa(5, str);
	CHECK_EQUAL(1, 1);
}

int main(int ac, char** av)
{
	const char *argv[] = { "exe", "-ojunit" };
	return CommandLineTestRunner::RunAllTests(sizeof(argv)/sizeof(const char *), argv);
}

extern "C" {
void assert_failed(unsigned char* file, unsigned int line)
{
  /*simple stub*/
}
}

IMPORT_TEST_GROUP(Util);

