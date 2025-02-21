import gc

def format_bytes(size):
    """Convert bytes to a human-readable format (KB, MB)."""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size < 1024:
            return f"{size:.2f} {unit}"
        size /= 1024
    return f"{size:.2f} GB"

def check_ram():
    """Check free and total RAM in a human-readable format."""
    gc.collect()  # Run garbage collection to free up unused memory
    free_mem = gc.mem_free()  # Get available free RAM
    total_mem = gc.mem_alloc() + free_mem  # Get total allocated + free RAM
    
    print(f"Total RAM: {format_bytes(total_mem)}")
    print(f"Free RAM:  {format_bytes(free_mem)}")
    print(f"Used RAM:  {format_bytes(total_mem - free_mem)}")

# Example usage
check_ram()
