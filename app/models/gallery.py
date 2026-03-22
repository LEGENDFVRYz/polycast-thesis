from app import db
from .mixins import SoftDeleteMixin


class Gallery(db.Model, SoftDeleteMixin):
    """
    Represents a gallery that hosts multiple sessions
    """
    __tablename__ = "galleries"
    
    id          = db.Column('id', db.Integer, primary_key=True)
    admin_id    = db.Column(db.Integer, db.ForeignKey('admins.id', ondelete='CASCADE'), nullable=False)     # Foreign Key to User (Many Gallery to One Admin)
    name        = db.Column(db.String(80), nullable=False)
    description = db.Column(db.String(255), nullable=True, default='')
    is_favorite = db.Column(db.Boolean, nullable=False, default=False, server_default='0')
    
    created_at          = db.Column(db.DateTime, nullable=False, default=db.func.now())
    last_activity_at    = db.Column(db.DateTime, nullable=True)     # updated when a session is added or recovered
    last_activity_type  = db.Column(db.String(32), nullable=True)   # ex. 'session_added', 'gallery_recovered'
    
    
    #-------------------------------------------
    # Relationships
    #-------------------------------------------
    sessions = db.relationship(
        'Session',
        backref='gallery',
        lazy=True,
        cascade="all, delete-orphan",
        passive_deletes=True
    )
    
    #-------------------------------------------
    # Constraints
    #-------------------------------------------
    __table_args__ = (
        db.UniqueConstraint('admin_id', 'name', name='uq_admin_gallery_name'),
        db.Index('idx_gallery_admin',           'admin_id'),                        # Fast lookup of galleries per admin
        db.Index('idx_gallery_last_activity',   'last_activity_at'),                # Fast sorting by recent activity
    )
    
    
    #-------------------------------------------
    # Getter Methods 
    #-------------------------------------------
    def to_dict(self):
        return {
            "id": self.id,
            "name": self.name,
            "admin_id": self.admin_id
        }
    
    def get_gallery_name(self):
        return self.name
    
    
    def record_activity(self, activity_type: str):
        """
        Convenience method to stamp last_activity_at and last_activity_type.
        - NOTE: Does NOT commit — caller is responsible for db.session.commit().
        """
        self.last_activity_at   = db.func.now()
        self.last_activity_type = activity_type
        