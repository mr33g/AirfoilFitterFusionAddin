import os
import uuid
import hashlib
import xml.etree.ElementTree as ET
from xml.dom import minidom

# Directories to exclude from the installer (don't install files from these)
EXCLUDED_DIRS = {
    'setup',        # Don't include the setup folder (contains MSI, build artifacts)
    '.git',         # Don't include git repository
    '.vscode',      # Don't include VS Code settings
    '__pycache__',  # Don't include Python cache (but we DO need to clean it up on uninstall)
    '.idea',        # Don't include JetBrains IDE settings
}

# Directories that may be created at runtime and need cleanup on uninstall
RUNTIME_CLEANUP_DIRS = {
    '__pycache__',  # Python bytecode cache - created at runtime
}

# File patterns to exclude
EXCLUDED_FILES = {
    '.gitignore',
    '.DS_Store',
    'Thumbs.db',
}

def make_id(prefix, path):
    """Creates a valid WiX ID, hashing it if it exceeds the 72-char limit."""
    safe_path = path.replace(os.sep, "_").replace(".", "_").replace("-", "_")
    full_id = f"{prefix}_{safe_path}"
    if len(full_id) <= 72:
        return full_id
    
    # Hash the path and append to prefix for a unique, short ID
    hash_str = hashlib.md5(path.encode()).hexdigest()[:16]
    return f"{prefix}_{hash_str}"

def should_exclude_dir(dirpath, source_dir):
    """Check if a directory should be excluded."""
    rel_path = os.path.relpath(dirpath, source_dir)
    parts = rel_path.split(os.sep)
    
    # Check if any part of the path is in excluded dirs
    for part in parts:
        if part in EXCLUDED_DIRS:
            return True
    return False

def generate_wxs_fragment(source_dir, output_file, component_group_id, directory_id_root):
    """
    Generates a WiX v4+ fragment (.wxs) for all files in a source directory.
    """
    # Use the WiX v4+ namespace
    root = ET.Element("Wix", xmlns="http://wixtoolset.org/schemas/v4/wxs")
    fragment = ET.SubElement(root, "Fragment")
    
    # Track directories to avoid duplicates
    dir_map = {".": directory_id_root, "": directory_id_root}
    
    # Track __pycache__ directories that need cleanup (they're created at runtime)
    pycache_dirs = set()
    
    # 1. Define Directory Structure
    # We walk once to collect all directories (including __pycache__ for cleanup purposes)
    for dirpath, dirnames, filenames in os.walk(source_dir):
        rel_path = os.path.relpath(dirpath, source_dir)
        
        # Skip fully excluded directories (but NOT __pycache__ - we need those for cleanup)
        if should_exclude_dir(dirpath, source_dir):
            # Check if this is a __pycache__ directory - we need to track these for cleanup
            if os.path.basename(dirpath) == '__pycache__':
                pycache_dirs.add(rel_path)
                dir_id = make_id("DIR", rel_path)
                dir_map[rel_path] = dir_id
            continue
            
        # Prune excluded directories from walk (but still record __pycache__ paths above)
        original_dirnames = list(dirnames)
        dirnames[:] = [d for d in dirnames if d not in EXCLUDED_DIRS]
        
        # Track __pycache__ subdirectories for cleanup
        for d in original_dirnames:
            if d == '__pycache__':
                pycache_rel = os.path.join(rel_path, d) if rel_path not in [".", ""] else d
                pycache_dirs.add(pycache_rel)
                pycache_id = make_id("DIR", pycache_rel)
                dir_map[pycache_rel] = pycache_id
            
        if rel_path in [".", ""]:
            continue
            
        dir_id = make_id("DIR", rel_path)
        dir_map[rel_path] = dir_id
    
    # 2. Components and Files
    comp_group = ET.SubElement(fragment, "ComponentGroup", Id=component_group_id)
    
    # Track directories that need RemoveFolder elements (to clean up on uninstall)
    dirs_needing_cleanup = set()
    
    file_count = 0
    for dirpath, dirnames, filenames in os.walk(source_dir):
        # Skip excluded directories
        if should_exclude_dir(dirpath, source_dir):
            continue
            
        # Prune excluded directories from walk
        dirnames[:] = [d for d in dirnames if d not in EXCLUDED_DIRS]
            
        rel_path = os.path.relpath(dirpath, source_dir)
        dir_id = dir_map.get(rel_path, directory_id_root)
        
        # Track this directory for cleanup (except root)
        if rel_path not in [".", ""]:
            dirs_needing_cleanup.add(rel_path)
        
        for filename in filenames:
            # Skip excluded files
            if filename in EXCLUDED_FILES:
                continue
                
            file_path = os.path.join(dirpath, filename)
            rel_file_path = os.path.relpath(file_path, source_dir)
            
            # Create valid WiX IDs
            comp_id = make_id("CMP", rel_file_path)
            file_id = make_id("FILE", rel_file_path)
            
            # Create Component
            comp = ET.SubElement(comp_group, "Component", 
                                 Id=comp_id, 
                                 Guid=str(uuid.uuid4()).upper(), 
                                 Directory=dir_id)
            
            # Registry value for per-user install consistency (required by WiX for per-user)
            ET.SubElement(comp, "RegistryValue", 
                                Root="HKCU", 
                                Key=f"Software\\AirfoilFitterFusionAddIn\\Files\\{file_id}", 
                                Name="installed", 
                                Type="integer", 
                                Value="1", 
                                Action="write")
            
            # File entry - path relative to setup folder (one level up)
            ET.SubElement(comp, "File", 
                          Id=file_id, 
                          Source=f"../{rel_file_path.replace(os.sep, '/')}", 
                          KeyPath="yes")
            
            file_count += 1
    
    # 2b. Add RemoveFolder components for each directory to ensure clean uninstall
    # Sort directories by depth (deepest first) to ensure proper removal order
    sorted_dirs = sorted(dirs_needing_cleanup, key=lambda p: p.count(os.sep), reverse=True)
    
    for rel_path in sorted_dirs:
        dir_id = dir_map.get(rel_path, directory_id_root)
        remove_comp_id = make_id("RMDIR", rel_path)
        
        # Create a component specifically for removing this directory
        remove_comp = ET.SubElement(comp_group, "Component",
                                    Id=remove_comp_id,
                                    Guid=str(uuid.uuid4()).upper(),
                                    Directory=dir_id)
        
        # Registry value as KeyPath (required for per-user components)
        ET.SubElement(remove_comp, "RegistryValue",
                      Root="HKCU",
                      Key=f"Software\\AirfoilFitterFusionAddIn\\Folders\\{remove_comp_id}",
                      Name="installed",
                      Type="integer",
                      Value="1",
                      Action="write")
        
        # RemoveFolder element to remove directory on uninstall
        ET.SubElement(remove_comp, "RemoveFolder",
                      Id=f"RF_{make_id('', rel_path)[-60:]}",  # Ensure ID is unique and within limits
                      On="uninstall")
    
    # 2c. Add cleanup components for __pycache__ directories (runtime-generated)
    # These directories contain .pyc files created by Python at runtime
    for pycache_path in sorted(pycache_dirs):
        pycache_dir_id = dir_map.get(pycache_path)
        if not pycache_dir_id:
            continue
            
        cleanup_comp_id = make_id("PYCACHE", pycache_path)
        
        # Create a component to clean up this __pycache__ directory
        cleanup_comp = ET.SubElement(comp_group, "Component",
                                     Id=cleanup_comp_id,
                                     Guid=str(uuid.uuid4()).upper(),
                                     Directory=pycache_dir_id)
        
        # Registry value as KeyPath (required for per-user components)
        ET.SubElement(cleanup_comp, "RegistryValue",
                      Root="HKCU",
                      Key=f"Software\\AirfoilFitterFusionAddIn\\Pycache\\{cleanup_comp_id}",
                      Name="installed",
                      Type="integer",
                      Value="1",
                      Action="write")
        
        # RemoveFile with wildcard to delete all .pyc files
        ET.SubElement(cleanup_comp, "RemoveFile",
                      Id=f"RMF_pyc_{make_id('', pycache_path)[-50:]}",
                      Name="*.pyc",
                      On="uninstall")
        
        # RemoveFile with wildcard to delete all .pyo files (older Python optimization files)
        ET.SubElement(cleanup_comp, "RemoveFile",
                      Id=f"RMF_pyo_{make_id('', pycache_path)[-50:]}",
                      Name="*.pyo",
                      On="uninstall")
        
        # RemoveFolder to remove the __pycache__ directory itself
        ET.SubElement(cleanup_comp, "RemoveFolder",
                      Id=f"RMD_pyc_{make_id('', pycache_path)[-50:]}",
                      On="uninstall")

    # 3. Define the directory structure in another fragment
    dir_fragment = ET.SubElement(root, "Fragment")
    
    # WiX v4+ uses Directory elements nested inside DirectoryRef
    for rel_path, dir_id in dir_map.items():
        if rel_path in [".", ""]:
            continue
        
        parent_rel = os.path.dirname(rel_path)
        parent_id = dir_map.get(parent_rel, directory_id_root)
        
        p_ref = ET.SubElement(dir_fragment, "DirectoryRef", Id=parent_id)
        ET.SubElement(p_ref, "Directory", Id=dir_id, Name=os.path.basename(rel_path))

    # Write XML
    xml_str = minidom.parseString(ET.tostring(root)).toprettyxml(indent="    ")
    with open(output_file, "w", encoding="utf-8") as f:
        f.write(xml_str)
    
    print(f"Generated {output_file} (WiX v4+) with {file_count} files.")

if __name__ == "__main__":
    generate_wxs_fragment(
        source_dir="..",
        output_file="Files.wxs",
        component_group_id="HarvestedAddInFiles",
        directory_id_root="ContentsFolder"
    )
