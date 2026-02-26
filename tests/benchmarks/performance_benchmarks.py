#!/usr/bin/env python3
"""
Performance Benchmarks and Validation Framework
Measures and validates system performance metrics.
"""

import time
import statistics
import psutil
import numpy as np
from dataclasses import dataclass, field
from typing import List, Dict, Callable, Optional, Any
from datetime import datetime
import json
import threading
import queue


@dataclass
class PerformanceMetrics:
    """Performance measurement results."""
    metric_name: str
    values: List[float] = field(default_factory=list)
    unit: str = ""
    timestamp: str = field(default_factory=lambda: datetime.now().isoformat())
    
    @property
    def mean(self) -> float:
        return statistics.mean(self.values) if self.values else 0.0
    
    @property
    def std_dev(self) -> float:
        return statistics.stdev(self.values) if len(self.values) > 1 else 0.0
    
    @property
    def min_val(self) -> float:
        return min(self.values) if self.values else 0.0
    
    @property
    def max_val(self) -> float:
        return max(self.values) if self.values else 0.0
    
    def to_dict(self) -> Dict[str, Any]:
        return {
            'metric_name': self.metric_name,
            'unit': self.unit,
            'mean': self.mean,
            'std_dev': self.std_dev,
            'min': self.min_val,
            'max': self.max_val,
            'samples': len(self.values),
            'timestamp': self.timestamp
        }


class PerformanceBenchmark:
    """Base class for performance benchmarks."""
    
    def __init__(self, name: str, duration: float = 60.0):
        self.name = name
        self.duration = duration
        self.metrics: Dict[str, PerformanceMetrics] = {}
        self.running = False
        
    def measure(self) -> Dict[str, PerformanceMetrics]:
        """Run benchmark and return metrics."""
        raise NotImplementedError
    
    def record_metric(self, name: str, value: float, unit: str = ""):
        """Record a metric value."""
        if name not in self.metrics:
            self.metrics[name] = PerformanceMetrics(metric_name=name, unit=unit)
        self.metrics[name].values.append(value)


class LatencyBenchmark(PerformanceBenchmark):
    """Measure system latency."""
    
    def __init__(self, name: str = "Latency Benchmark"):
        super().__init__(name, duration=30.0)
        self.callback_times: queue.Queue = queue.Queue()
        
    def measure_callback_latency(self, callback: Callable, iterations: int = 1000) -> PerformanceMetrics:
        """Measure callback execution latency."""
        latencies = []
        
        for _ in range(iterations):
            start = time.perf_counter_ns()
            callback()
            end = time.perf_counter_ns()
            
            latency_us = (end - start) / 1000.0
            latencies.append(latency_us)
        
        return PerformanceMetrics(
            metric_name="callback_latency",
            values=latencies,
            unit="us"
        )
    
    def measure_ros_latency(self, topic_name: str, duration: float = 10.0) -> PerformanceMetrics:
        """Measure ROS topic latency."""
        # Would subscribe to topic and measure message delays
        return PerformanceMetrics(
            metric_name="ros_latency",
            values=[1.0],  # Placeholder
            unit="ms"
        )
    
    def measure(self) -> Dict[str, PerformanceMetrics]:
        """Run latency benchmarks."""
        print(f"\nRunning {self.name}...")
        
        # Test function latency
        def test_func():
            x = np.random.random(100)
            return np.sum(x)
        
        latency_metric = self.measure_callback_latency(test_func, iterations=10000)
        self.metrics['function_latency'] = latency_metric
        
        print(f"  Function latency: {latency_metric.mean:.2f} ± {latency_metric.std_dev:.2f} us")
        
        return self.metrics


class ThroughputBenchmark(PerformanceBenchmark):
    """Measure system throughput."""
    
    def __init__(self, name: str = "Throughput Benchmark"):
        super().__init__(name)
        
    def measure(self) -> Dict[str, PerformanceMetrics]:
        """Run throughput benchmarks."""
        print(f"\nRunning {self.name}...")
        
        # Message throughput test
        msg_count = 0
        start_time = time.time()
        duration = 10.0
        
        while time.time() - start_time < duration:
            msg_count += 1
            time.sleep(0.0001)  # Simulate processing
        
        throughput = msg_count / duration
        self.record_metric('message_throughput', throughput, 'msgs/sec')
        
        print(f"  Message throughput: {throughput:.0f} msgs/sec")
        
        # Data throughput test
        data_sizes = []
        start_time = time.time()
        
        while time.time() - start_time < duration:
            data = np.random.bytes(1024 * 100)  # 100 KB
            data_sizes.append(len(data))
        
        data_throughput = sum(data_sizes) / duration / (1024 * 1024)  # MB/s
        self.record_metric('data_throughput', data_throughput, 'MB/s')
        
        print(f"  Data throughput: {data_throughput:.2f} MB/s")
        
        return self.metrics


class ResourceBenchmark(PerformanceBenchmark):
    """Measure resource utilization."""
    
    def __init__(self, name: str = "Resource Benchmark"):
        super().__init__(name)
        self.process = psutil.Process()
        
    def measure(self) -> Dict[str, PerformanceMetrics]:
        """Run resource benchmarks."""
        print(f"\nRunning {self.name}...")
        
        cpu_samples = []
        memory_samples = []
        
        start_time = time.time()
        while time.time() - start_time < self.duration:
            cpu_samples.append(self.process.cpu_percent())
            memory_samples.append(self.process.memory_info().rss / (1024 * 1024))  # MB
            time.sleep(0.1)
        
        self.metrics['cpu_usage'] = PerformanceMetrics(
            metric_name='cpu_usage',
            values=cpu_samples,
            unit='percent'
        )
        
        self.metrics['memory_usage'] = PerformanceMetrics(
            metric_name='memory_usage',
            values=memory_samples,
            unit='MB'
        )
        
        print(f"  CPU usage: {self.metrics['cpu_usage'].mean:.1f}%")
        print(f"  Memory usage: {self.metrics['memory_usage'].mean:.1f} MB")
        
        return self.metrics


class SimulationBenchmark(PerformanceBenchmark):
    """Benchmark simulation performance."""
    
    def __init__(self, name: str = "Simulation Benchmark"):
        super().__init__(name, duration=30.0)
        
    def measure(self) -> Dict[str, PerformanceMetrics]:
        """Run simulation benchmarks."""
        print(f"\nRunning {self.name}...")
        
        # Real-time factor test
        rt_factors = []
        step_times = []
        
        dt = 0.001  # 1ms simulation step
        start_time = time.time()
        sim_time = 0.0
        
        while time.time() - start_time < self.duration:
            step_start = time.time()
            
            # Simulate physics step
            sim_time += dt
            time.sleep(dt * 0.5)  # Simulate 50% load
            
            step_end = time.time()
            step_duration = step_end - step_start
            step_times.append(step_duration * 1000)  # ms
            
            rt_factor = dt / step_duration if step_duration > 0 else 1.0
            rt_factors.append(rt_factor)
        
        self.metrics['real_time_factor'] = PerformanceMetrics(
            metric_name='real_time_factor',
            values=rt_factors,
            unit='ratio'
        )
        
        self.metrics['physics_step_time'] = PerformanceMetrics(
            metric_name='physics_step_time',
            values=step_times,
            unit='ms'
        )
        
        print(f"  Real-time factor: {self.metrics['real_time_factor'].mean:.2f}x")
        print(f"  Physics step time: {self.metrics['physics_step_time'].mean:.2f} ms")
        
        return self.metrics


class ValidationSuite:
    """Validate system against requirements."""
    
    def __init__(self):
        self.requirements: Dict[str, Dict] = {}
        self.validations: List[Dict] = []
        
    def add_requirement(self, req_id: str, description: str, 
                        metric: str, threshold: float, 
                        comparison: str = 'less_than'):
        """Add a requirement to validate."""
        self.requirements[req_id] = {
            'description': description,
            'metric': metric,
            'threshold': threshold,
            'comparison': comparison
        }
    
    def validate(self, metrics: Dict[str, PerformanceMetrics]) -> List[Dict]:
        """Validate metrics against requirements."""
        results = []
        
        for req_id, req in self.requirements.items():
            metric_name = req['metric']
            
            if metric_name not in metrics:
                results.append({
                    'req_id': req_id,
                    'status': 'NOT_TESTED',
                    'message': f'Metric {metric_name} not available'
                })
                continue
            
            value = metrics[metric_name].mean
            threshold = req['threshold']
            
            if req['comparison'] == 'less_than':
                passed = value < threshold
            elif req['comparison'] == 'greater_than':
                passed = value > threshold
            elif req['comparison'] == 'equal':
                passed = abs(value - threshold) < 0.001
            else:
                passed = False
            
            results.append({
                'req_id': req_id,
                'description': req['description'],
                'metric': metric_name,
                'value': value,
                'threshold': threshold,
                'status': 'PASSED' if passed else 'FAILED'
            })
        
        self.validations = results
        return results


class BenchmarkRunner:
    """Run all benchmarks and generate reports."""
    
    def __init__(self):
        self.benchmarks: List[PerformanceBenchmark] = []
        self.validation_suite = ValidationSuite()
        
    def add_benchmark(self, benchmark: PerformanceBenchmark):
        """Add a benchmark to run."""
        self.benchmarks.append(benchmark)
    
    def run_all(self) -> Dict[str, Any]:
        """Run all benchmarks."""
        print("\n" + "="*60)
        print("PERFORMANCE BENCHMARK SUITE")
        print("="*60)
        
        all_metrics: Dict[str, PerformanceMetrics] = {}
        
        for benchmark in self.benchmarks:
            metrics = benchmark.measure()
            all_metrics.update(metrics)
        
        # Validate requirements
        validations = self.validation_suite.validate(all_metrics)
        
        # Generate report
        report = {
            'timestamp': datetime.now().isoformat(),
            'metrics': {name: m.to_dict() for name, m in all_metrics.items()},
            'validations': validations,
            'summary': {
                'total_benchmarks': len(self.benchmarks),
                'total_metrics': len(all_metrics),
                'passed_requirements': sum(1 for v in validations if v['status'] == 'PASSED'),
                'failed_requirements': sum(1 for v in validations if v['status'] == 'FAILED')
            }
        }
        
        # Print summary
        print("\n" + "="*60)
        print("BENCHMARK SUMMARY")
        print("="*60)
        for name, metric in all_metrics.items():
            print(f"{name}: {metric.mean:.2f} {metric.unit}")
        
        print(f"\nRequirements: {report['summary']['passed_requirements']}/"
              f"{len(validations)} passed")
        
        return report
    
    def save_report(self, report: Dict[str, Any], filename: str = "benchmark_report.json"):
        """Save report to file."""
        with open(filename, 'w') as f:
            json.dump(report, f, indent=2)
        print(f"\nReport saved to: {filename}")


def main():
    """Run performance benchmarks."""
    runner = BenchmarkRunner()
    
    # Add benchmarks
    runner.add_benchmark(LatencyBenchmark())
    runner.add_benchmark(ThroughputBenchmark())
    runner.add_benchmark(ResourceBenchmark())
    runner.add_benchmark(SimulationBenchmark())
    
    # Add requirements
    runner.validation_suite.add_requirement(
        'LAT-001', 'Callback latency < 100us',
        'function_latency', 100.0, 'less_than'
    )
    runner.validation_suite.add_requirement(
        'THR-001', 'Message throughput > 1000 msgs/sec',
        'message_throughput', 1000.0, 'greater_than'
    )
    runner.validation_suite.add_requirement(
        'SIM-001', 'Real-time factor > 0.9',
        'real_time_factor', 0.9, 'greater_than'
    )
    
    # Run benchmarks
    report = runner.run_all()
    runner.save_report(report)
    
    return report


if __name__ == '__main__':
    main()
