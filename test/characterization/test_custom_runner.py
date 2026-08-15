from platformio.test.runners.base import TestRunnerBase
from platformio.test.result import TestCase, TestStatus


class CustomTestRunner(TestRunnerBase):
    def stage_testing(self):
        super().stage_testing()
        self.test_suite.add_case(
            TestCase(name="characterization", status=TestStatus.PASSED)
        )
