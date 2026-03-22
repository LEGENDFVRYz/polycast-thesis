from app import db
from .mixins import SoftDeleteMixin
from datetime import datetime, UTC, timedelta


class Session(db.Model, SoftDeleteMixin):
    """
    Represents an session of notes per gallery
    """
    __tablename__ = "sessions"
    
    id          = db.Column('id', db.Integer, primary_key=True)
    gallery_id  = db.Column(db.Integer, db.ForeignKey('galleries.id', ondelete='CASCADE'), nullable=False)   # Foreign Key to Gallery (Many Sessions to One Gallery)
    name        = db.Column(db.String(80), nullable=False)
    created_at  = db.Column(db.DateTime, default=db.func.now(), nullable=False)
    
    last_activity_at    = db.Column(db.DateTime, nullable=True)     
    last_activity_type  = db.Column(db.String(32), nullable=True)    # x. 'session_added', 'session_recovered'
    
    
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
        db.Index('idx_session_gallery',         'gallery_id'),                      # Fast lookup of sessions per gallery
        db.Index('idx_session_last_activity',   'last_activity_at'),                # Fast sorting by recent activity
    )
    
    
    #-------------------------------------------
    # Helper Methods
    #-------------------------------------------
    def record_activity(self, activity_type: str):
        """
        Stamp last_activity_at and last_activity_type.
        - NOTE: Does NOT commit — caller is responsible for db.session.commit().
 
        Accepted values:
          'ACTIVE'    — session created, or a note was added/removed
          'RECOVERED' — session was restored from the bin
        """
        self.last_activity_at   = db.func.now()
        self.last_activity_type = activity_type

    
    @property
    def is_new(self) -> bool:
        """True if the session was created within the last 7 days."""
        if not self.created_at:
            return False
        return (datetime.utcnow() - self.created_at) <= timedelta(days=7)
 

    @property
    def is_recently_updated(self) -> bool:
        """True if last_activity_at was updated within the last 7 days (but session is not brand-new)."""
        if not self.last_activity_at or self.is_new:
            return False
        return (datetime.utcnow() - self.last_activity_at) <= timedelta(days=7)
        