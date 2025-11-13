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