<<<<<<< HEAD
import os
import csv
from datetime import datetime
from sqlalchemy import create_engine, Column, Integer, String, Boolean, DateTime, LargeBinary, ForeignKey, UniqueConstraint
from sqlalchemy.ext.declarative import declarative_base
from sqlalchemy.orm import sessionmaker, scoped_session

# --- Database Setup ---
DB_FILE = 'instance/app.db'
DB_DIR = os.path.dirname(DB_FILE)

# Ensure the 'instance' directory exists
if DB_DIR and not os.path.exists(DB_DIR):
    os.makedirs(DB_DIR)
    print(f"Created directory: {DB_DIR}")

DATABASE_URL = f"sqlite:///{DB_FILE}"

engine = create_engine(DATABASE_URL)
session_factory = sessionmaker(autocommit=False, autoflush=False, bind=engine)
SessionLocal = scoped_session(session_factory)
Base = declarative_base()


# --- Model Definitions ---
# We redefine the models here to make the script standalone.
# These definitions match the structure you provided.

class Admin(Base):
    __tablename__ = "admins"
    id = Column(Integer, primary_key=True)
    username = Column(String(80), unique=True, nullable=False)
    # Store the bcrypt hash as bytes, as per your model
    password_hash = Column(LargeBinary(128), nullable=False)

class Gallery(Base):
    __tablename__ = "galleries"
    id = Column(Integer, primary_key=True)
    admin_id = Column(Integer, ForeignKey('admins.id', ondelete='CASCADE'), nullable=False)
    name = Column(String(80), nullable=False)
    is_favorite = Column(Boolean, nullable=False, default=False)
    
    # This field comes from your SoftDeleteMixin
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


# --- Data Processing Functions ---

def import_admins(db):
    """Imports admins from admins.csv"""
    print("Processing admins.csv...")
    try:
        with open('instance/admins.csv', mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                # Check if admin already exists
                exists = db.query(Admin).get(int(row['id']))
                if exists:
                    continue # Skip if already imported
                    
                admin = Admin(
                    id=int(row['id']),
                    username=row['username'],
                    # Convert the string hash from CSV to bytes for LargeBinary
                    password_hash=row['password_hash'].encode('utf-8')
                )
                db.add(admin)
            db.commit()
        print(f"Successfully imported {db.query(Admin).count()} admins.")
    except Exception as e:
        print(f"Error importing admins: {e}")
        db.rollback()
        raise

def import_galleries(db):
    """Imports galleries from galleries.csv"""
    print("Processing galleries.csv...")
    try:
        with open('instance/galleries.csv', mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                exists = db.query(Gallery).get(int(row['id']))
                if exists:
                    continue

                # Handle the deleted_at flag from the CSV
                # 0 -> None (not deleted)
                # 1 -> Current timestamp (soft deleted)
                deleted_time = None
                if row['deleted_at'] == '1':
                    deleted_time = datetime.utcnow()

                gallery = Gallery(
                    id=int(row['id']),
                    admin_id=int(row['admin_id']),
                    name=row['name'],
                    # Convert '0' or '1' to boolean
                    is_favorite=bool(int(row['is_favorite'])),
                    deleted_at=deleted_time
                )
                db.add(gallery)
            db.commit()
        print(f"Successfully imported {db.query(Gallery).count()} galleries.")
    except Exception as e:
        print(f"Error importing galleries: {e}")
        db.rollback()
        raise

def import_sessions(db):
    """Imports sessions from sessions.csv"""
    print("Processing sessions.csv...")
    try:
        with open('instance/sessions.csv', mode='r', encoding='utf-8') as f:
            reader = csv.DictReader(f)
            for row in reader:
                exists = db.query(Session).get(int(row['id']))
                if exists:
                    continue
                    
                session = Session(
                    id=int(row['id']),
                    gallery_id=int(row['gallery_id']),
                    name=row['name'],
                    # Convert date string to datetime object
                    created_at=datetime.strptime(row['created_at'], '%Y-%m-%d %H:%M:%S')
                )
                db.add(session)
            db.commit()
        print(f"Successfully imported {db.query(Session).count()} sessions.")
    except Exception as e:
        print(f"Error importing sessions: {e}")
        db.rollback()
        raise

# --- Main Execution ---

def main():
    print(f"Initializing database at {DB_FILE}...")
    # Create all tables
    Base.metadata.create_all(bind=engine)
    print("Tables created (if they didn't exist).")
    
    db = SessionLocal()
    
    try:
        # Import in order of dependency
        import_admins(db)
        import_galleries(db)
        import_sessions(db)
        
        print("\n--- Data import successful! ---")
        
    except Exception as e:
        print(f"\n--- An error occurred! ---")
        print(f"Details: {e}")
        print("Database transaction has been rolled back.")
        
    finally:
        SessionLocal.remove()
        print("Database session closed.")

if __name__ == "__main__":
    main()
=======
import csv
import os
import random
import shutil
from datetime import datetime, timedelta
from app import create_app, db 
from app.models.admin import Admin 
from app.models.gallery import Gallery
from app.models.session import Session
from app.models.note import Note  # Added Note model

# Initialize App
app = create_app() 

# Configuration
ARCHIVE_ROOT = 'archive'
SEED_DATA = 'seed/seed_data.csv'
SOURCE_ROOT = 'instance/source_images'
SOURCE_FOLDERS = ['v1', 'v2', 'v3', 'v4']

def seed_database():
    filename = SEED_DATA
    
    # Check if inputs exist
    if not os.path.exists(filename):
        print(f"Error: {filename} not found.")
        return
    
    if not os.path.exists(SOURCE_ROOT):
        print(f"Error: Source directory '{SOURCE_ROOT}' not found. Please create it and add v1-v4 folders.")
        return

    # Ensure root archive directory exists
    if not os.path.exists(ARCHIVE_ROOT):
        os.makedirs(ARCHIVE_ROOT)
        print(f"Created root directory: {ARCHIVE_ROOT}")

    print("--- Starting Seeder ---")
    
    # Caches
    admin_cache = {}
    gallery_cache = {}
    session_cache = set()

    with open(filename, mode='r', encoding='utf-8') as csv_file:
        csv_reader = csv.DictReader(csv_file)
        
        row_count = 0
        notes_created_count = 0
        
        for row in csv_reader:
            username = row['username'].strip()
            gallery_name = row['gallery_name'].strip()
            session_name = row['session_name'].strip()

            # --------------------------
            # 1. Handle Admin
            # --------------------------
            admin = admin_cache.get(username)
            if not admin:
                admin = Admin.query.filter_by(username=username).first()
                if not admin:
                    print(f"Creating Admin: {username}")
                    admin = Admin(username=username)
                    admin.set_password("test123") 
                    db.session.add(admin)
                    db.session.flush()
                admin_cache[username] = admin

            # --------------------------
            # 2. Handle Gallery
            # --------------------------
            gallery_key = (admin.id, gallery_name)
            gallery = gallery_cache.get(gallery_key)
            if not gallery:
                gallery = Gallery.query.filter_by(admin_id=admin.id, name=gallery_name).first()
                if not gallery:
                    gallery = Gallery(name=gallery_name, admin_id=admin.id)
                    db.session.add(gallery)
                    db.session.flush()
                gallery_cache[gallery_key] = gallery

            # --------------------------
            # 3. Handle Session & Folders
            # --------------------------
            session_key = (gallery.id, session_name)
            
            # Only process if we haven't seen this session in this specific run
            if session_key not in session_cache:
                existing_session = Session.query.filter_by(gallery_id=gallery.id, name=session_name).first()
                
                current_session = None
                is_new_session = False

                if not existing_session:
                    is_new_session = True
                    # Create DB Record
                    days_back = random.randint(0, 14)
                    seconds_offset = random.randint(0, 86399)
                    random_date = datetime.now() - timedelta(days=days_back, seconds=seconds_offset)
                    
                    current_session = Session(
                        name=session_name, 
                        gallery_id=gallery.id,
                        created_at=random_date
                    )
                    db.session.add(current_session)
                    db.session.flush() # Generate ID
                else:
                    current_session = existing_session

                session_cache.add(session_key)

                # --------------------------
                # 4. Handle Notes (Images)
                # --------------------------
                # Only seed notes if we just created this session (prevents duplicating images on re-runs)
                if is_new_session:
                    
                    # A. Create Target Directory
                    target_dir = os.path.join(
                        ARCHIVE_ROOT,
                        str(admin.id),
                        str(gallery.id),
                        str(current_session.id)
                    )
                    os.makedirs(target_dir, exist_ok=True)

                    # B. Select Random Source Folder
                    selected_v_folder = random.choice(SOURCE_FOLDERS)
                    source_path = os.path.join(SOURCE_ROOT, selected_v_folder)
                    
                    if os.path.exists(source_path):
                        # Get all files in source folder
                        files = [f for f in os.listdir(source_path) if os.path.isfile(os.path.join(source_path, f))]
                        # Sort to ensure deterministic order if needed, or shuffle for randomness. 
                        # Here we just iterate them.
                        files.sort() 

                        page_counter = 1
                        
                        for file_name in files:
                            # Logic: 
                            # 1. Create Note DB record
                            # 2. Copy file to target renamed as note_{page_number}.png/.jpg
                            
                            # Create DB Record
                            new_note = Note(
                                session_id=current_session.id,
                                page_number=page_counter
                            )
                            db.session.add(new_note)
                            
                            # Determine extension (keep original extension or force png based on your preference)
                            # Your Note model getter implies "note_{page}.png", so we might need to convert or just rename.
                            # For this script, I will respect the source extension but rename the file.
                            _, ext = os.path.splitext(file_name)
                            # If your model strictly expects .png in the getter, ensure source is png or update model.
                            # Assuming we align with model getter:
                            target_filename = f"note_{page_counter}{ext}" 
                            
                            src_file_full = os.path.join(source_path, file_name)
                            dst_file_full = os.path.join(target_dir, target_filename)
                            
                            shutil.copy2(src_file_full, dst_file_full)
                            
                            page_counter += 1
                            notes_created_count += 1
                    else:
                        print(f"Warning: Source folder {source_path} does not exist. Skipping images for this session.")

            row_count += 1

    # Commit all DB changes
    try:
        db.session.commit()
        print(f"--- Success! Processed {row_count} rows. ---")
        print(f"Admins: {len(admin_cache)}")
        print(f"Galleries: {len(gallery_cache)}")
        print(f"Notes Created: {notes_created_count}")
    except Exception as e:
        db.session.rollback()
        print(f"--- Error occurred. Rollback executed. ---")
        print(str(e))

if __name__ == "__main__":
    with app.app_context():
        seed_database()
>>>>>>> fdb13409200eafb42d614b4da78471ab7f92cc32
