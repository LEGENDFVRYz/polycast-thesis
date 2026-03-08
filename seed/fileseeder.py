import os
import csv
import shutil
import random
import re
from datetime import datetime
from sqlalchemy import create_engine, Column, Integer, String, Boolean, DateTime, LargeBinary, ForeignKey, UniqueConstraint
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, scoped_session

# --- Configuration ---
DB_FILE = 'instance/app.db'
DB_DIR = os.path.dirname(DB_FILE)
DATABASE_URL = f"sqlite:///{DB_FILE}"

# --- New Configuration for Folders & Images ---
# The folder where your master images are stored
SOURCE_IMAGE_FOLDER = 'source_images' 
# The root folder for the new structure
OUTPUT_ROOT_FOLDER = 'data'
# The min/max number of images to copy into EACH folder
IMAGES_PER_SESSION_MIN = 40
IMAGES_PER_SESSION_MAX = 90


# --- Database Setup ---
if DB_DIR and not os.path.exists(DB_DIR):
    os.makedirs(DB_DIR)
    print(f"Created directory: {DB_DIR}")

engine = create_engine(DATABASE_URL)
session_factory = sessionmaker(autocommit=False, autoflush=False, bind=engine)
SessionLocal = scoped_session(session_factory)
Base = declarative_base()


# --- Model Definitions (Unchanged) ---
class Admin(Base):
    __tablename__ = "admins"
    id = Column(Integer, primary_key=True)
    username = Column(String(80), unique=True, nullable=False)
    password_hash = Column(LargeBinary(128), nullable=False)

class Gallery(Base):
    __tablename__ = "galleries"
    id = Column(Integer, primary_key=True)
    admin_id = Column(Integer, ForeignKey('admins.id', ondelete='CASCADE'), nullable=False)
    name = Column(String(80), nullable=False)
    is_favorite = Column(Boolean, nullable=False, default=False)
    deleted_at = Column(DateTime, nullable=True, default=None)
    __table_args__ = (
        UniqueConstraint('admin_id', 'name', name='uq_admin_gallery_name'),
    )

class Session(Base):
    __tablename__ = "sessions"
    id = Column(Integer, primary_key=True)
    gallery_id = Column(Integer, ForeignKey('galleries.id', ondelete='CASCADE'), nullable=False)
    name = Column(String(80), nullable=False)
    created_at = Column(DateTime, nullable=False)
    __table_args__ = (
        UniqueConstraint('gallery_id', 'name', name='uq_gallery_session_name'),
    )


# --- Data Import Functions (Using 'instance/' path) ---

def import_admins(db):
    """Imports admins from instance/admins.csv"""
    print("Processing instance/admins.csv...")
    try:
        with open('instance/admins.csv', mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            count = 0
            for row in reader:
                exists = db.query(Admin).get(int(row['id']))
                if exists:
                    continue
                admin = Admin(
                    id=int(row['id']),
                    username=row['username'],
                    password_hash=row['password_hash'].encode('utf-8')
                )
                db.add(admin)
                count += 1
            db.commit()
        print(f"Successfully imported {count} new admins.")
    except Exception as e:
        print(f"Error importing admins: {e}")
        db.rollback()
        raise

def import_galleries(db):
    """Imports galleries from instance/galleries.csv"""
    print("Processing instance/galleries.csv...")
    try:
        with open('instance/galleries.csv', mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            count = 0
            for row in reader:
                exists = db.query(Gallery).get(int(row['id']))
                if exists:
                    continue
                deleted_time = None
                if row['deleted_at'] == '1':
                    deleted_time = datetime.utcnow()
                gallery = Gallery(
                    id=int(row['id']),
                    admin_id=int(row['admin_id']),
                    name=row['name'],
                    is_favorite=bool(int(row['is_favorite'])),
                    deleted_at=deleted_time
                )
                db.add(gallery)
                count += 1
            db.commit()
        print(f"Successfully imported {count} new galleries.")
    except Exception as e:
        print(f"Error importing galleries: {e}")
        db.rollback()
        raise

def import_sessions(db):
    """Imports sessions from instance/sessions.csv"""
    print("Processing instance/sessions.csv...")
    try:
        with open('instance/sessions.csv', mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            count = 0
            for row in reader:
                exists = db.query(Session).get(int(row['id']))
                if exists:
                    continue
                session = Session(
                    id=int(row['id']),
                    gallery_id=int(row['gallery_id']),
                    name=row['name'],
                    created_at=datetime.strptime(row['created_at'], '%Y-%m-%d %H:%M:%S')
                )
                db.add(session)
                count += 1
            db.commit()
        print(f"Successfully imported {count} new sessions.")
    except Exception as e:
        print(f"Error importing sessions: {e}")
        db.rollback()
        raise


# --- New Folder & Image Functions ---

def sanitize_name(name):
    """Replaces spaces and removes chars invalid for directory names."""
    name = name.replace(' ', '_')
    # Remove any characters that aren't alphanumeric, underscore, hyphen, or dot
    name = re.sub(r'[^\w\-_\.]', '', name)
    return name.strip()

# --- THIS FUNCTION IS MODIFIED ---
def create_folders_and_copy_images(db):
    """
    Queries the DB for all sessions and creates the new folder structure,
    copying random images into each session folder.
    """
    print("\n--- Starting Folder & Image Creation ---")
    
    # 1. Check for source images
    if not os.path.exists(SOURCE_IMAGE_FOLDER):
        print(f"WARNING: Source image folder '{SOURCE_IMAGE_FOLDER}' not found. Skipping image copy.")
        return
        
    try:
        all_source_images = [
            f for f in os.listdir(SOURCE_IMAGE_FOLDER) 
            if os.path.isfile(os.path.join(SOURCE_IMAGE_FOLDER, f))
        ]
    except Exception as e:
        print(f"ERROR: Could not read source images from '{SOURCE_IMAGE_FOLDER}'. {e}")
        return

    if not all_source_images:
        print(f"WARNING: No images found in '{SOURCE_IMAGE_FOLDER}'. Skipping image copy.")
        return
        
    print(f"Found {len(all_source_images)} images in '{SOURCE_IMAGE_FOLDER}'.")

    # 2. Query the database for all paths
    # We join Session -> Gallery -> Admin to get the required IDs and name
    paths_query = db.query(
        Admin.username,
        Session.id,
        Session.gallery_id # This is the Gallery.id
    ).join(
        Gallery, Session.gallery_id == Gallery.id
    ).join(
        Admin, Gallery.admin_id == Admin.id
    ).all()
    
    if not paths_query:
        print("No sessions found in database to create folders for.")
        return

    print(f"Found {len(paths_query)} session paths to create...")
    created_count = 0

    # 3. Loop, create folders, and copy images
    for admin_name, session_id, gallery_id in paths_query:
        
        # Sanitize admin name
        safe_admin = sanitize_name(admin_name)
        # Convert IDs to strings for path
        str_session_id = str(session_id)
        str_gallery_id = str(gallery_id)
        
        # Create the full nested path based on the new structure:
        # OUTPUT_ROOT_FOLDER / admin_name / session_id / gallery_id
        target_dir = os.path.join(OUTPUT_ROOT_FOLDER, safe_admin, str_session_id, str_gallery_id)
        
        try:
            os.makedirs(target_dir, exist_ok=True)
            
            # 4. Copy random images
            # Get the number of images to copy
            num_to_copy = random.randint(IMAGES_PER_SESSION_MIN, IMAGES_PER_SESSION_MAX)
            
            # Use random.choices (with replacement) to build the list of images.
            # This ensures we get 40-90 images even if source_images is small.
            images_to_copy_list = random.choices(all_source_images, k=num_to_copy)
            
            for i, img_name in enumerate(images_to_copy_list):
                source_path = os.path.join(SOURCE_IMAGE_FOLDER, img_name)
                
                # Create a unique name for the destination file
                # e.g., 001_original_name.jpg, 002_original_name.jpg
                base, ext = os.path.splitext(img_name)
                dest_name = f"{i:03d}_{sanitize_name(base)}{ext}"
                dest_path = os.path.join(target_dir, dest_name)
                
                # Copy the file
                shutil.copy2(source_path, dest_path)
                
            created_count += 1
            
        except Exception as e:
            print(f"ERROR creating folder or copying to '{target_dir}': {e}")
            
    print(f"--- Successfully created {created_count} session folders and copied images. ---")


# --- Main Execution ---

def main():
    print(f"Initializing database at {DB_FILE}...")
    Base.metadata.create_all(bind=engine)
    print("Tables created (if they didn't exist).")
    
    db = SessionLocal()
    
    try:
        # 1. Import all data
        import_admins(db)
        import_galleries(db)
        import_sessions(db)
        
        print("\n--- Database import successful! ---")
        
        # 2. Create folders and copy images
        create_folders_and_copy_images(db)
        
    except Exception as e:
        print(f"\n--- An error occurred during database import! ---")
        print(f"Details: {e}")
        print("Database transaction has been rolled back. Folder creation was skipped.")
        
    finally:
        SessionLocal.remove()
        print("\nDatabase session closed.")

if __name__ == "__main__":
    main()