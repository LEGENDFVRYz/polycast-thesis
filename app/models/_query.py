# app/models/query.py
from app import db

class SoftDeleteQuery(db.Query):
    """
    A custom query class for models with soft delete.
    Provides methods like .with_trashed() and .only_trashed().
    """
    
    def with_trashed(self):
        """
        Returns a new query that includes soft-deleted items.
        It does this by returning a base db.Query, bypassing
        the default filter.
        """
        # self._model_class is the model (e.g., Gallery)
        return db.Query(self._model_class, session=self.session)

    def only_trashed(self):
        """
        Returns a new query that *only* includes soft-deleted items.
        """
        return self.with_trashed().filter(self._model_class.deleted_at != None)