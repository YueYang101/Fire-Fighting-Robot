#!/usr/bin/env python3
"""
Enhanced Code File Scanner - Can save full content to file
python code_scanner.py --save-full
"""

import os
import sys
from pathlib import Path
from datetime import datetime

# Define code file extensions by category
CODE_EXTENSIONS = {
    'Python': ['.py', '.pyw', '.pyx', '.pyi', '.pyc'],
    'JavaScript/TypeScript': ['.js', '.jsx', '.ts', '.tsx', '.mjs', '.cjs'],
    'Web': ['.html', '.htm', '.css', '.scss', '.sass', '.less'],
    'Java': ['.java', '.class', '.jar'],
    'C/C++': ['.c', '.cpp', '.cc', '.cxx', '.h', '.hpp', '.hh', '.hxx'],
    'C#': ['.cs', '.csx'],
    'Go': ['.go'],
    'Rust': ['.rs'],
    'Ruby': ['.rb', '.erb'],
    'PHP': ['.php', '.phtml'],
    'Swift': ['.swift'],
    'Kotlin': ['.kt', '.kts'],
    'Scala': ['.scala'],
    'Shell': ['.sh', '.bash', '.zsh', '.fish', '.ps1', '.bat', '.cmd'],
    'SQL': ['.sql', '.mysql', '.pgsql', '.sqlite'],
    'R': ['.r', '.R', '.Rmd'],
    'MATLAB': ['.m', '.mat'],
    'Julia': ['.jl'],
    'Lua': ['.lua'],
    'Perl': ['.pl', '.pm'],
    'Config': ['.json', '.xml', '.yaml', '.yml', '.toml', '.ini', '.cfg', '.conf'],
    'Documentation': ['.md', '.rst', '.txt', '.adoc'],
    'Build': ['Makefile', 'CMakeLists.txt', '.gradle', '.maven', 'pom.xml'],
    'Docker': ['Dockerfile', '.dockerignore', 'docker-compose.yml'],
    'Git': ['.gitignore', '.gitattributes', '.gitmodules'],
    'Other': ['.vim', '.el', '.lisp', '.clj', '.dart', '.asm', '.s']
}

# Directories to skip
SKIP_DIRS = {
    '__pycache__', 'venv', 'env', '.env', 'virtualenv',
    'node_modules', '.git', '.svn', '.hg',
    '.idea', '.vscode', '.vs',
    'target', 'build', 'dist', 'out',
    '.pytest_cache', '.mypy_cache',
    'vendor', 'packages', '.bundle',
    'coverage', '.coverage', 'htmlcov',
    '.next', '.nuxt', '.cache'
}

# Binary extensions to skip when reading content
BINARY_EXTENSIONS = {'.pyc', '.pyo', '.so', '.dll', '.dylib', '.exe', '.class', '.jar', '.war', '.ear'}

def get_all_code_extensions():
    """Get a set of all code extensions"""
    extensions = set()
    for ext_list in CODE_EXTENSIONS.values():
        extensions.update(ext_list)
    return extensions

def is_code_file(filepath):
    """Check if a file is a code file"""
    path = Path(filepath)
    
    # Check special filenames
    special_files = {'Makefile', 'Dockerfile', 'CMakeLists.txt', 'pom.xml', 'docker-compose.yml'}
    if path.name in special_files:
        return True
    
    # Check extensions
    all_extensions = get_all_code_extensions()
    return path.suffix.lower() in all_extensions

def get_file_category(filepath):
    """Get the category of a code file"""
    path = Path(filepath)
    
    for category, extensions in CODE_EXTENSIONS.items():
        if path.suffix.lower() in extensions or path.name in extensions:
            return category
    
    return "Unknown"

def format_size(size):
    """Format file size in human-readable form"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size < 1024.0:
            return f"{size:.1f} {unit}"
        size /= 1024.0
    return f"{size:.1f} TB"

def count_lines(filepath):
    """Count lines in a file"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            return sum(1 for _ in f)
    except:
        try:
            with open(filepath, 'r', encoding='latin-1') as f:
                return sum(1 for _ in f)
        except:
            return 0

def read_file_safely(filepath, max_size=1024*1024):  # 1MB limit
    """Read file content safely"""
    try:
        # Skip binary files
        if Path(filepath).suffix.lower() in BINARY_EXTENSIONS:
            return "[Binary file - content not displayed]"
        
        # Check file size
        size = os.path.getsize(filepath)
        if size > max_size:
            return f"[File too large - {format_size(size)} - content not displayed]"
        
        # Skip the scanner script itself
        if os.path.basename(filepath) == os.path.basename(__file__):
            return "[Scanner script - content not displayed]"
        
        # Try to read file
        try:
            with open(filepath, 'r', encoding='utf-8') as f:
                return f.read()
        except UnicodeDecodeError:
            with open(filepath, 'r', encoding='latin-1') as f:
                return f.read()
                
    except Exception as e:
        return f"[Error reading file: {e}]"

def get_directory_tree(directory=".", prefix="", max_depth=5, current_depth=0, show_all=False):
    """Generate directory tree structure"""
    output = []
    
    if current_depth >= max_depth:
        return output
    
    try:
        path = Path(directory)
        items = list(path.iterdir())
        
        # Filter items
        if not show_all:
            items = [item for item in items if not item.name.startswith('.') or item.name in {'.gitignore', '.dockerignore'}]
            items = [item for item in items if item.is_file() or (item.is_dir() and item.name not in SKIP_DIRS)]
        
        items.sort(key=lambda x: (x.is_file(), x.name.lower()))
        
        for index, item in enumerate(items):
            is_last = index == len(items) - 1
            current_prefix = "└── " if is_last else "├── "
            
            if item.is_file() and is_code_file(item):
                size = format_size(item.stat().st_size)
                output.append(f"{prefix}{current_prefix}{item.name} ({size})")
            elif item.is_dir() and item.name not in SKIP_DIRS:
                output.append(f"{prefix}{current_prefix}{item.name}/")
                extension = "    " if is_last else "│   "
                output.extend(get_directory_tree(item, prefix + extension, max_depth, current_depth + 1, show_all))
    
    except PermissionError:
        output.append(f"{prefix}└── [Permission Denied]")
    
    return output

def scan_code_files(base_path=".", show_content=True, max_files=None, save_full_content=False):
    """Scan for all code files in directory"""
    
    print("=" * 80)
    print("CODE FILE SCANNER")
    print(f"Scanning: {os.path.abspath(base_path)}")
    print(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 80)
    print()
    
    # Show directory structure
    print("DIRECTORY STRUCTURE (code files only):")
    print("-" * 40)
    tree = get_directory_tree(base_path)
    for line in tree:
        print(line)
    print()
    
    # Collect all code files
    code_files = []
    total_size = 0
    total_lines = 0
    
    for root, dirs, files in os.walk(base_path):
        # Skip certain directories
        dirs[:] = [d for d in dirs if d not in SKIP_DIRS]
        
        for file in files:
            full_path = os.path.join(root, file)
            if is_code_file(full_path) and os.path.basename(full_path) != os.path.basename(__file__):
                rel_path = os.path.relpath(full_path, base_path)
                size = os.path.getsize(full_path)
                lines = count_lines(full_path)
                category = get_file_category(full_path)
                
                code_files.append({
                    'path': rel_path,
                    'full_path': full_path,
                    'size': size,
                    'lines': lines,
                    'category': category
                })
                
                total_size += size
                total_lines += lines
    
    # Sort files by category and then by path
    code_files.sort(key=lambda x: (x['category'], x['path']))
    
    # Show statistics
    print("=" * 80)
    print("STATISTICS:")
    print("-" * 40)
    print(f"Total code files: {len(code_files)}")
    print(f"Total size: {format_size(total_size)}")
    print(f"Total lines: {total_lines:,}")
    print()
    
    # Show files by category
    print("FILES BY CATEGORY:")
    print("-" * 40)
    
    category_stats = {}
    for file in code_files:
        cat = file['category']
        if cat not in category_stats:
            category_stats[cat] = {'count': 0, 'size': 0, 'lines': 0}
        category_stats[cat]['count'] += 1
        category_stats[cat]['size'] += file['size']
        category_stats[cat]['lines'] += file['lines']
    
    for category in sorted(category_stats.keys()):
        stats = category_stats[category]
        print(f"{category}: {stats['count']} files, {format_size(stats['size'])}, {stats['lines']:,} lines")
    print()
    
    # Show file contents if requested
    if show_content:
        print("=" * 80)
        print("FILE CONTENTS:")
        print("=" * 80)
        
        files_to_show = code_files[:max_files] if max_files else code_files
        
        for file_info in files_to_show:
            print(f"\n{'=' * 80}")
            print(f"FILE: {file_info['path']}")
            print(f"Category: {file_info['category']} | Size: {format_size(file_info['size'])} | Lines: {file_info['lines']}")
            print(f"{'=' * 80}")
            
            content = read_file_safely(file_info['full_path'])
            print(content)
            print(f"{'=' * 80}\n")
        
        if max_files and len(code_files) > max_files:
            print(f"\n[Showing first {max_files} files out of {len(code_files)} total]")
    
    # Save summary to file
    summary_file = "code_scan_summary.txt"
    with open(summary_file, 'w') as f:
        f.write(f"Code Scan Summary - {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"Directory: {os.path.abspath(base_path)}\n")
        f.write(f"Total files: {len(code_files)}\n")
        f.write(f"Total size: {format_size(total_size)}\n")
        f.write(f"Total lines: {total_lines:,}\n\n")
        
        f.write("Files by category:\n")
        for category in sorted(category_stats.keys()):
            stats = category_stats[category]
            f.write(f"  {category}: {stats['count']} files\n")
        
        f.write("\nAll code files:\n")
        for file_info in code_files:
            f.write(f"  {file_info['path']} ({file_info['category']}, {format_size(file_info['size'])})\n")
    
    print(f"\nSummary saved to: {summary_file}")
    
    # Save full content if requested
    if save_full_content:
        full_content_file = "code_full_content.txt"
        with open(full_content_file, 'w', encoding='utf-8') as f:
            f.write("=" * 80 + "\n")
            f.write("FULL CODE CONTENT DUMP\n")
            f.write(f"Directory: {os.path.abspath(base_path)}\n")
            f.write(f"Time: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write("=" * 80 + "\n\n")
            
            # Write directory structure
            f.write("DIRECTORY STRUCTURE:\n")
            f.write("-" * 40 + "\n")
            for line in tree:
                f.write(line + "\n")
            f.write("\n")
            
            # Write statistics
            f.write("=" * 80 + "\n")
            f.write("STATISTICS:\n")
            f.write("-" * 40 + "\n")
            f.write(f"Total code files: {len(code_files)}\n")
            f.write(f"Total size: {format_size(total_size)}\n")
            f.write(f"Total lines: {total_lines:,}\n\n")
            
            f.write("FILES BY CATEGORY:\n")
            f.write("-" * 40 + "\n")
            for category in sorted(category_stats.keys()):
                stats = category_stats[category]
                f.write(f"{category}: {stats['count']} files, {format_size(stats['size'])}, {stats['lines']:,} lines\n")
            f.write("\n")
            
            # Write all file contents
            f.write("=" * 80 + "\n")
            f.write("FILE CONTENTS:\n")
            f.write("=" * 80 + "\n")
            
            for file_info in code_files:
                f.write(f"\n{'=' * 80}\n")
                f.write(f"FILE: {file_info['path']}\n")
                f.write(f"Category: {file_info['category']} | Size: {format_size(file_info['size'])} | Lines: {file_info['lines']}\n")
                f.write(f"{'=' * 80}\n")
                
                content = read_file_safely(file_info['full_path'])
                f.write(content)
                f.write(f"\n{'=' * 80}\n\n")
        
        print(f"Full content saved to: {full_content_file}")

def main():
    """Main function"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Scan directory for code files')
    parser.add_argument('path', nargs='?', default='.', help='Directory to scan (default: current directory)')
    parser.add_argument('--no-content', action='store_true', help='Skip displaying file contents')
    parser.add_argument('--max-files', type=int, help='Maximum number of files to display content for')
    parser.add_argument('--save-full', action='store_true', help='Save full file contents to code_full_content.txt')
    
    args = parser.parse_args()
    
    if not os.path.exists(args.path):
        print(f"Error: Path '{args.path}' does not exist")
        sys.exit(1)
    
    scan_code_files(
        base_path=args.path,
        show_content=not args.no_content,
        max_files=args.max_files,
        save_full_content=args.save_full
    )

if __name__ == "__main__":
    main()