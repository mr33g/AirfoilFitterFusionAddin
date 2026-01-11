#!/usr/bin/env python
"""
Helper script to copy source files to staging directory for building.
Excludes lib, setup, .git, __pycache__, and other build artifacts.
"""
import os
import shutil
import sys
import fnmatch

# Directories to exclude from staging
EXCLUDED_DIRS = {
    'lib',           # Dependencies - will be installed separately
    'setup',         # Build scripts and artifacts
    '.git',          # Git repository
    '.github',       # GitHub workflows and CI/CD files
    '__pycache__',   # Python cache
    '.vscode',       # IDE settings
    '.idea',         # IDE settings
    '.pytest_cache', # Test cache
    'dist',          # Distribution artifacts
    'build',         # Build artifacts
    'doc',           # Papers
    'res',           # Test data
    'out',           # Output data
}

# Files to exclude
EXCLUDED_FILES = {
    '.gitignore',
    'requirements.txt',  # Only needed for building, not runtime
    '.DS_Store',
    'Thumbs.db',
    '*.pyc',
    '*.pyo',
    '*.pyd',
}

def should_exclude_path(path, root_dir):
    """Check if a path should be excluded from staging."""
    rel_path = os.path.relpath(path, root_dir)
    parts = rel_path.split(os.sep)
    
    # Check if any part of the path is in excluded dirs
    for part in parts:
        if part in EXCLUDED_DIRS:
            return True
    
    # Check if file should be excluded
    filename = os.path.basename(path)
    if filename in EXCLUDED_FILES:
        return True
    
    # Check patterns
    for pattern in EXCLUDED_FILES:
        if '*' in pattern:
            if fnmatch.fnmatch(filename, pattern):
                return True
    
    return False

def copy_to_staging(source_dir, staging_dir):
    """Copy source files to staging directory, excluding build artifacts."""
    if not os.path.exists(source_dir):
        print(f"ERROR: Source directory does not exist: {source_dir}")
        return False
    
    if os.path.exists(staging_dir):
        shutil.rmtree(staging_dir)
    os.makedirs(staging_dir)
    
    for root, dirs, files in os.walk(source_dir):
        # Filter out excluded directories
        dirs[:] = [d for d in dirs if not should_exclude_path(os.path.join(root, d), source_dir)]
        
        # Calculate relative path
        rel_root = os.path.relpath(root, source_dir)
        
        # Skip if root itself should be excluded
        if rel_root != '.' and should_exclude_path(root, source_dir):
            continue
        
        # Create corresponding directory in staging
        if rel_root == '.':
            staging_subdir = staging_dir
        else:
            staging_subdir = os.path.join(staging_dir, rel_root)
            os.makedirs(staging_subdir, exist_ok=True)
        
        # Copy files
        for filename in files:
            source_file = os.path.join(root, filename)
            if should_exclude_path(source_file, source_dir):
                continue
            
            staging_file = os.path.join(staging_subdir, filename)
            try:
                shutil.copy2(source_file, staging_file)
            except Exception as e:
                print(f"WARNING: Failed to copy {source_file}: {e}")
    
    return True

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: stage_for_build.py <source_dir> <staging_dir>")
        sys.exit(1)
    
    source_dir = os.path.abspath(sys.argv[1])
    staging_dir = os.path.abspath(sys.argv[2])
    
    if not copy_to_staging(source_dir, staging_dir):
        sys.exit(1)
    
    print(f"Successfully staged files from {source_dir} to {staging_dir}")
