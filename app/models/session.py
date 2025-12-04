from app import db
from .mixins import SoftDeleteMixin


class Session(db.Model, SoftDeleteMixin):
    """
    Represents an session of notes per gallery
    """
    __tablename__ = "sessions"
    
    id          = db.Column('id', db.Integer, primary_key=True)
    gallery_id  = db.Column(db.Integer, db.ForeignKey('galleries.id', ondelete='CASCADE'), nullable=False)   # Foreign Key to Gallery (Many Sessions to One Gallery)
    name        = db.Column(db.String(128), nullable=False)
    created_at  = db.Column(db.DateTime, default=db.func.now(), nullable=False)
    
    
    # -------------------------------------------
    # Relationships
    # -------------------------------------------
    notes = db.relationship(
        'Note',
        backref='session',
        lazy='dynamic', 
        cascade="all, delete-orphan",
        passive_deletes=True,
        order_by="Note.page_number" 
    )
    
    
    #-------------------------------------------
    # Constraints
    #-------------------------------------------
    __table_args__ = (
        db.UniqueConstraint('gallery_id', 'name', name='uq_gallery_session_name'),
    )
    
    
    
    