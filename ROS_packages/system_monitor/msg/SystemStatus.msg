# Complete System Status Message
std_msgs/Header header
string hostname
float32 cpu_temperature
float32 gpu_temperature
SystemLoad system_load
NetworkStats network_stats
DiskUsage[] disk_usage
ProcessInfo[] top_cpu_processes
ProcessInfo[] top_memory_processes
int32 process_count
int32 thread_count
float32 uptime_hours
