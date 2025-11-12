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
    is_favorite = db.Column(db.Boolean, nullable=False, default=False, server_default='0')
    
    
    
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
