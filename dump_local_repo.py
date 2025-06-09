#!/usr/bin/env python3
import os
import sys
from pathlib import Path

def print_file_content(file_path, base_path):
    """Print the content of a file with proper formatting."""
    relative_path = os.path.relpath(file_path, base_path)
    print(f"\n{'='*80}")
    print(f"FILE: {relative_path}")
    print(f"{'='*80}")
    
    try:
        with open(file_path, 'r', encoding='utf-8') as f:
            content = f.read()
            print(content)
    except Exception as e:
        print(f"ERROR reading file: {e}")
    
    print(f"{'='*80}\n")

def scan_directory(directory):
    """Recursively scan directory and print all file contents."""
    base_path = Path(directory).resolve()
    
    # Define file extensions to include
    extensions_to_include = [
        '.py', '.yaml', '.yml', '.xml', '.txt', '.cmake', 
        '.msg', '.srv', '.action', '.launch', '.sh', '.bash'
    ]
    
    # Define directories to skip
    dirs_to_skip = ['__pycache__', '.git', 'build', 'install', 'log', '.vscode', '.idea']
    
    print(f"Scanning directory: {base_path}")
    print(f"{'='*80}")
    
    # First, print the directory structure
    print("\nDIRECTORY STRUCTURE:")
    print("-" * 40)
    
    for root, dirs, files in os.walk(base_path):
        # Skip certain directories
        dirs[:] = [d for d in dirs if d not in dirs_to_skip]
        
        level = root.replace(str(base_path), '').count(os.sep)
        indent = ' ' * 2 * level
        print(f"{indent}{os.path.basename(root)}/")
        
        subindent = ' ' * 2 * (level + 1)
        for file in sorted(files):
            print(f"{subindent}{file}")
    
    print("\n" + "="*80)
    print("FILE CONTENTS:")
    print("="*80)
    
    # Now print file contents
    for root, dirs, files in os.walk(base_path):
        # Skip certain directories
        dirs[:] = [d for d in dirs if d not in dirs_to_skip]
        
        for file in sorted(files):
            file_path = os.path.join(root, file)
            
            # Check if file has an extension we want to include
            if any(file.endswith(ext) for ext in extensions_to_include) or file in ['CMakeLists.txt', 'package.xml', 'setup.py', 'setup.cfg']:
                print_file_content(file_path, base_path)

def main():
    # Look for mlx90640_driver in current directory or subdirectories
    current_dir = os.getcwd()
    
    # Search for mlx90640_driver directory
    mlx_dir = None
    for root, dirs, files in os.walk(current_dir):
        if 'mlx90640_driver' in dirs:
            mlx_dir = os.path.join(root, 'mlx90640_driver')
            break
    
    if not mlx_dir:
        print("Error: Could not find mlx90640_driver directory")
        print(f"Current directory: {current_dir}")
        print("\nPlease run this script from your Fire-fighting-Robot repository root")
        sys.exit(1)
    
    print(f"Found mlx90640_driver at: {mlx_dir}")
    scan_directory(mlx_dir)

if __name__ == "__main__":
    main()