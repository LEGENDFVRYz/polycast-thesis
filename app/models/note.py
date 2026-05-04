from app import db
import os

class Note(db.Model):
    """
    Represents an individual image file within a session.
    Read-only ImagePath; scalable for inserting metadata in the future.
    """
    __tablename__ = "notes"

    id = db.Column(db.Integer, primary_key=True)
    session_id = db.Column(db.Integer, db.ForeignKey('sessions.id', ondelete='CASCADE'), nullable=False, index=True)
    
    page_number = db.Column(db.Integer, nullable=False) # Used for logical sorting, and file naming
    
    # Metadata columns: additional info like width/height or generated timestamp
    file_size_bytes = db.Column(db.Integer, nullable=True)
    
    
    
    # -------------------------------------------
    # Constraints & Indexes
    # -------------------------------------------
    __table_args__ = (
        # Ensure a specific file path doesn't exist twice within the same session
        db.UniqueConstraint('session_id', 'page_number', name='uq_session_filename'),
        # specific index for fast retrieval of sorted images per session
        db.Index('idx_session_page', 'session_id', 'page_number'),
    )
    
    
    #-------------------------------------------
    # Getter Methods 
    #-------------------------------------------
    @property
    def filename(self):
        """
        Derive filename from page_number.
        """
        return f"note_{self.page_number}.jpg"
    

    @property
    def filepath(self):
        """
        Derive full path dynamically.
        """
        
        # Safety check first
        if not self.session or not self.session.gallery or not self.session.gallery.admin:
            return None
        
        return "/".join([
            str(self.session.gallery.admin_id), 
            str(self.session.gallery.id), 
            str(self.session.id), 
            self.filename
        ])