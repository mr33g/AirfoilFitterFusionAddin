import os
import uuid
import hashlib
import xml.etree.ElementTree as ET
from xml.dom import minidom

# Directories to exclude from the installer
EXCLUDED_DIRS = {
    'setup',        # Don't include the setup folder (contains MSI, build artifacts)
    '.git',         # Don't include git repository
    '.vscode',      # Don't include VS Code settings
    '__pycache__',  # Don't include Python cache
    '.idea',        # Don't include JetBrains IDE settings
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
    
    # 1. Define Directory Structure
    # We walk once to collect all directories
    for dirpath, dirnames, filenames in os.walk(source_dir):
        # Skip excluded directories
        if should_exclude_dir(dirpath, source_dir):
            continue
            
        # Prune excluded directories from walk
        dirnames[:] = [d for d in dirnames if d not in EXCLUDED_DIRS]
            
        rel_path = os.path.relpath(dirpath, source_dir)
        if rel_path in [".", ""]:
            continue
            
        dir_id = make_id("DIR", rel_path)
        dir_map[rel_path] = dir_id
    
    # 2. Components and Files
    comp_group = ET.SubElement(fragment, "ComponentGroup", Id=component_group_id)
    
    file_count = 0
    for dirpath, dirnames, filenames in os.walk(source_dir):
        # Skip excluded directories
        if should_exclude_dir(dirpath, source_dir):
            continue
            
        # Prune excluded directories from walk
        dirnames[:] = [d for d in dirnames if d not in EXCLUDED_DIRS]
            
        rel_path = os.path.relpath(dirpath, source_dir)
        dir_id = dir_map.get(rel_path, directory_id_root)
        
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
