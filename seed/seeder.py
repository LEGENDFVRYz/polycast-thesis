import csv
import os
import random
import shutil
from datetime import datetime, timedelta, UTC
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
                    # Give the gallery a random created_at within the last 30 days
                    gallery_created = datetime.now(UTC) - timedelta(
                        days=random.randint(7, 30),
                        seconds=random.randint(0, 86399)
                    )
                    gallery = Gallery(
                        name=gallery_name,
                        admin_id=admin.id,
                        created_at=gallery_created,
                    )
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

                    # Random created_at: somewhere between gallery.created_at and now
                    days_back = random.randint(0, 14)
                    seconds_offset = random.randint(0, 86399)
                    session_created = datetime.now(UTC) - timedelta(days=days_back, seconds=seconds_offset)

                    # last_activity_at: a small random offset after created_at (simulates notes being added soon after)
                    activity_offset = timedelta(minutes=random.randint(5, 120))
                    session_activity = session_created + activity_offset

                    current_session = Session(
                        name=session_name, 
                        gallery_id=gallery.id,
                        created_at=session_created,
                        last_activity_at=session_activity,
                        last_activity_type='ACTIVE',    # Newly created session — default to ACTIVE
                    )
                    db.session.add(current_session)
                    db.session.flush() # Generate ID

                    # Bubble latest session activity up to the parent gallery
                    if gallery.last_activity_at is None or session_activity > gallery.last_activity_at:
                        gallery.last_activity_at   = session_activity
                        gallery.last_activity_type = 'session_added'

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
                            
                            _, ext = os.path.splitext(file_name)
                            target_filename = f"note_{page_counter}{ext}" 
                            
                            src_file_full = os.path.join(source_path, file_name)
                            dst_file_full = os.path.join(target_dir, target_filename)
                            
                            shutil.copy2(src_file_full, dst_file_full)

                            # Capture file size in bytes after copy (populates Note.file_size_bytes metadata)
                            file_size = os.path.getsize(dst_file_full)

                            # Create DB Record
                            new_note = Note(
                                session_id=current_session.id,
                                page_number=page_counter,
                                file_size_bytes=file_size,
                            )
                            db.session.add(new_note)
                            
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