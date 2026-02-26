#!/usr/bin/env python3
"""
Test Suite Runner
Orchestrates all test scenarios and generates reports.
"""

import sys
import time
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Optional, Callable
from dataclasses import dataclass, asdict
from enum import Enum

# Import test scenarios
sys.path.insert(0, str(Path(__file__).parent))

try:
    from scenarios.test_takeoff import TakeoffTestNode, run_takeoff_test
    from scenarios.test_landing import LandingTestNode, run_landing_test
    from scenarios.test_waypoint import WaypointNavigator, run_waypoint_test
    from scenarios.test_obstacle_avoidance import ObstacleAvoidanceNode, run_obstacle_test
except ImportError:
    pass


class TestStatus(Enum):
    PENDING = "pending"
    RUNNING = "running"
    PASSED = "passed"
    FAILED = "failed"
    ERROR = "error"
    SKIPPED = "skipped"


@dataclass
class TestResult:
    """Result of a single test."""
    name: str
    status: TestStatus
    duration: float
    message: str = ""
    metrics: Optional[Dict] = None
    timestamp: str = ""
    
    def __post_init__(self):
        if not self.timestamp:
            self.timestamp = datetime.now().isoformat()


@dataclass
class TestSuiteReport:
    """Complete test suite report."""
    suite_name: str
    start_time: str
    end_time: str
    total_tests: int
    passed: int
    failed: int
    errors: int
    skipped: int
    results: List[TestResult]
    summary: Dict
    
    def to_dict(self) -> Dict:
        return {
            'suite_name': self.suite_name,
            'start_time': self.start_time,
            'end_time': self.end_time,
            'total_tests': self.total_tests,
            'passed': self.passed,
            'failed': self.failed,
            'errors': self.errors,
            'skipped': self.skipped,
            'results': [
                {
                    'name': r.name,
                    'status': r.status.value,
                    'duration': r.duration,
                    'message': r.message,
                    'metrics': r.metrics,
                    'timestamp': r.timestamp
                }
                for r in self.results
            ],
            'summary': self.summary
        }
    
    def to_json(self, indent: int = 2) -> str:
        return json.dumps(self.to_dict(), indent=indent)


class TestSuite:
    """Manages and runs test suites."""
    
    def __init__(self, name: str = "Robotics Test Suite"):
        self.name = name
        self.tests: Dict[str, Callable] = {}
        self.results: List[TestResult] = []
        
    def register_test(self, name: str, test_func: Callable):
        """Register a test function."""
        self.tests[name] = test_func
    
    def run_test(self, name: str) -> TestResult:
        """Run a single test."""
        if name not in self.tests:
            return TestResult(
                name=name,
                status=TestStatus.ERROR,
                duration=0.0,
                message=f"Test '{name}' not found"
            )
        
        print(f"\n{'='*60}")
        print(f"Running: {name}")
        print('='*60)
        
        start_time = time.time()
        
        try:
            result = self.tests[name]()
            duration = time.time() - start_time
            
            # Determine status from result
            if hasattr(result, 'success'):
                status = TestStatus.PASSED if result.success else TestStatus.FAILED
            elif isinstance(result, dict):
                status = TestStatus.PASSED if result.get('success', True) else TestStatus.FAILED
            else:
                status = TestStatus.PASSED
            
            # Extract metrics
            metrics = None
            if hasattr(result, '__dataclass_fields__'):
                metrics = asdict(result)
            elif isinstance(result, dict):
                metrics = result
            
            return TestResult(
                name=name,
                status=status,
                duration=duration,
                message="Test completed",
                metrics=metrics
            )
            
        except Exception as e:
            duration = time.time() - start_time
            return TestResult(
                name=name,
                status=TestStatus.ERROR,
                duration=duration,
                message=str(e)
            )
    
    def run_all(self) -> TestSuiteReport:
        """Run all registered tests."""
        start_time = datetime.now().isoformat()
        
        passed = failed = errors = skipped = 0
        
        for name in self.tests:
            result = self.run_test(name)
            self.results.append(result)
            
            if result.status == TestStatus.PASSED:
                passed += 1
            elif result.status == TestStatus.FAILED:
                failed += 1
            elif result.status == TestStatus.ERROR:
                errors += 1
            elif result.status == TestStatus.SKIPPED:
                skipped += 1
        
        end_time = datetime.now().isoformat()
        
        # Generate summary
        total = len(self.results)
        success_rate = passed / total if total > 0 else 0.0
        
        summary = {
            'success_rate': success_rate,
            'total_duration': sum(r.duration for r in self.results),
            'avg_duration': sum(r.duration for r in self.results) / total if total > 0 else 0.0,
            'test_categories': self._categorize_tests()
        }
        
        return TestSuiteReport(
            suite_name=self.name,
            start_time=start_time,
            end_time=end_time,
            total_tests=total,
            passed=passed,
            failed=failed,
            errors=errors,
            skipped=skipped,
            results=self.results,
            summary=summary
        )
    
    def _categorize_tests(self) -> Dict[str, int]:
        """Categorize tests by type."""
        categories = {}
        for result in self.results:
            # Extract category from name
            if 'takeoff' in result.name.lower():
                cat = 'takeoff'
            elif 'landing' in result.name.lower():
                cat = 'landing'
            elif 'waypoint' in result.name.lower():
                cat = 'waypoint'
            elif 'obstacle' in result.name.lower():
                cat = 'obstacle_avoidance'
            else:
                cat = 'other'
            
            categories[cat] = categories.get(cat, 0) + 1
        
        return categories
    
    def save_report(self, report: TestSuiteReport, output_dir: Path = None):
        """Save test report to file."""
        if output_dir is None:
            output_dir = Path(__file__).parent / 'reports'
        
        output_dir.mkdir(parents=True, exist_ok=True)
        
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        report_file = output_dir / f'test_report_{timestamp}.json'
        
        with open(report_file, 'w') as f:
            f.write(report.to_json())
        
        print(f"\nReport saved to: {report_file}")
        return report_file


def create_default_suite() -> TestSuite:
    """Create default test suite with all scenarios."""
    suite = TestSuite("Robotics Simulation Test Suite")
    
    # Register all test scenarios
    suite.register_test("UAV Takeoff", lambda: run_takeoff_test())
    suite.register_test("UAV Landing", lambda: run_landing_test())
    suite.register_test("UAV Waypoint Navigation", lambda: run_waypoint_test('uav'))
    suite.register_test("USV Waypoint Navigation", lambda: run_waypoint_test('usv'))
    suite.register_test("UGV Waypoint Navigation", lambda: run_waypoint_test('ugv'))
    suite.register_test("UGV Obstacle Avoidance", lambda: run_obstacle_test('ugv'))
    suite.register_test("UAV Obstacle Avoidance", lambda: run_obstacle_test('uav'))
    
    return suite


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(description='Run robotics test suite')
    parser.add_argument('--test', '-t', type=str, help='Run specific test')
    parser.add_argument('--list', '-l', action='store_true', help='List available tests')
    parser.add_argument('--output', '-o', type=str, help='Output directory for reports')
    
    args = parser.parse_args()
    
    suite = create_default_suite()
    
    if args.list:
        print("\nAvailable Tests:")
        for i, name in enumerate(suite.tests.keys(), 1):
            print(f"  {i}. {name}")
        return
    
    if args.test:
        # Run specific test
        result = suite.run_test(args.test)
        print(f"\n{'='*60}")
        print(f"Result: {result.status.value.upper()}")
        print(f"Duration: {result.duration:.2f}s")
        if result.message:
            print(f"Message: {result.message}")
        if result.metrics:
            print(f"Metrics: {json.dumps(result.metrics, indent=2)}")
    else:
        # Run all tests
        print(f"\n{'='*60}")
        print(f"Running {len(suite.tests)} tests...")
        print('='*60)
        
        report = suite.run_all()
        
        # Print summary
        print(f"\n{'='*60}")
        print("TEST SUITE SUMMARY")
        print('='*60)
        print(f"Total: {report.total_tests}")
        print(f"Passed: {report.passed} ✓")
        print(f"Failed: {report.failed} ✗")
        print(f"Errors: {report.errors} ⚠")
        print(f"Skipped: {report.skipped} ⊘")
        print(f"Success Rate: {report.summary['success_rate']*100:.1f}%")
        print(f"Total Duration: {report.summary['total_duration']:.2f}s")
        print('='*60)
        
        # Save report
        output_dir = Path(args.output) if args.output else None
        suite.save_report(report, output_dir)


if __name__ == '__main__':
    main()
