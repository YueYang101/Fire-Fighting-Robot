#!/usr/bin/env python3
"""
Script to scan robot-dashboard directory on Mac and output all file contents
Run this from inside the robot-dashboard directory
"""

import os
from pathlib import Path

def get_file_structure(directory=".", prefix="", max_depth=5, current_depth=0):
    """Get directory structure as string"""
    output = []
    
    if current_depth >= max_depth:
        return output
    
    try:
        items = sorted(list(Path(directory).iterdir()))
        dirs = [item for item in items if item.is_dir() and not item.name.startswith('.')]
        files = [item for item in items if item.is_file() and not item.name.startswith('.')]
        
        # Directories first, then files
        all_items = dirs + files
        
        for index, item in enumerate(all_items):
            is_last = index == len(all_items) - 1
            
            # Add tree branch
            output.append(prefix + ("└── " if is_last else "├── ") + item.name)
            
            # Recurse for directories
            if item.is_dir() and item.name not in ['__pycache__', 'venv', 'node_modules', '.git', '.idea']:
                extension = "    " if is_last else "│   "
                output.extend(get_file_structure(item, prefix + extension, max_depth, current_depth + 1))
    
    except PermissionError:
        output.append(prefix + "└── [Permission Denied]")
    
    return output

def read_file_safely(filepath):
    """Read file content safely"""
    try:
        # Skip binary files and this script itself
        if os.path.basename(filepath) == 'dashboard_scanner.py':
            return "[Skipping scanner script itself]"
            
        binary_extensions = ['.pyc', '.pyo', '.so', '.dylib', '.png', '.jpg', '.jpeg', '.gif', '.ico', '.pdf']
        if any(str(filepath).endswith(ext) for ext in binary_extensions):
            return "[Binary file - skipped]"
        
        with open(filepath, 'r', encoding='utf-8') as f:
            content = f.read()
            return content
    except UnicodeDecodeError:
        try:
            with open(filepath, 'r', encoding='latin-1') as f:
                return f.read()
        except:
            return "[Could not decode file]"
    except Exception as e:
        return f"[Error reading file: {e}]"

def scan_dashboard_complete(base_path="."):
    """Scan entire dashboard directory and output everything"""
    
    print("=" * 80)
    print("ROBOT DASHBOARD COMPLETE SCAN")
    print(f"Path: {os.path.abspath(base_path)}")
    print("=" * 80)
    print()
    
    # Get directory structure
    print("DIRECTORY STRUCTURE:")
    print("-" * 40)
    structure = get_file_structure(base_path)
    for line in structure:
        print(line)
    print()
    
    # Get all files
    all_files = []
    for root, dirs, files in os.walk(base_path):
        # Skip certain directories
        dirs[:] = [d for d in dirs if d not in ['__pycache__', 'venv', 'node_modules', '.git', '.idea']]
        
        for file in files:
            if not file.startswith('.') and not file.endswith('.pyc') and file != 'dashboard_scanner.py':
                full_path = os.path.join(root, file)
                rel_path = os.path.relpath(full_path, base_path)
                all_files.append((rel_path, full_path))
    
    # Sort files by path
    all_files.sort()
    
    print("=" * 80)
    print("FILE CONTENTS:")
    print("=" * 80)
    
    for rel_path, full_path in all_files:
        print(f"\n{'=' * 80}")
        print(f"FILE: {rel_path}")
        print(f"{'=' * 80}")
        
        content = read_file_safely(full_path)
        print(content)
        print(f"{'=' * 80}\n")

def main():
    # Run from current directory (should be robot-dashboard)
    current_dir = os.getcwd()
    if 'robot-dashboard' not in current_dir:
        print("Warning: This script should be run from inside the robot-dashboard directory")
        print(f"Current directory: {current_dir}")
        print("\nContinuing anyway...\n")
    
    # Scan current directory
    scan_dashboard_complete(".")

if __name__ == "__main__":
    main()