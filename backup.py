import sqlite3
import os
import datetime

# CONFIGURATION
DB_NAME = "instance/app.db"   # Ensure this path matches your project structure
BACKUP_FILE = "backup.sql"

def validate_and_backup():
    print(f"[{datetime.datetime.now()}] --- INITIALIZING DATABASE MAINTENANCE ---")
    
    if not os.path.exists(DB_NAME):
        print(f"CRITICAL ERROR: Database file '{DB_NAME}' not found.")
        return

    conn = sqlite3.connect(DB_NAME)
    cursor = conn.cursor()
    
    # ---------------------------------------------------------
    # STEP 1: DATA STATISTICS (LOGGING ROW COUNTS)
    # ---------------------------------------------------------
    print("\n--- PHASE 1: DATA STATISTICS ---")
    
    # List of your specific tables to check
    target_tables = ['admins', 'galleries', 'sessions', 'notes']
    table_counts = {}

    try:
        for table in target_tables:
            cursor.execute(f"SELECT COUNT(*) FROM {table}")
            count = cursor.fetchone()[0]
            table_counts[table] = count
            print(f" > Found {count:<5} records in table: '{table}'")
    except sqlite3.OperationalError as e:
        print(f" ! Warning: Could not read table stats. Reason: {e}")

    total_records = sum(table_counts.values())
    print(f" > TOTAL RECORDS TO ARCHIVE: {total_records}")

    # ---------------------------------------------------------
    # STEP 2: INTEGRITY CHECKS
    # ---------------------------------------------------------
    print("\n--- PHASE 2: VALIDATION CHECKS ---")
    
    # Check 1: Physical Integrity
    cursor.execute("PRAGMA integrity_check;")
    integrity = cursor.fetchone()[0]
    print(f" > Physical Integrity Check:  {integrity.upper()}")
    
    # Check 2: Logical (Foreign Key) Integrity
    cursor.execute("PRAGMA foreign_key_check;")
    fk_errors = cursor.fetchall()
    
    if len(fk_errors) == 0:
        print(" > Referential Integrity:   VALID (0 Orphans found)")
    else:
        print(f" ! CRITICAL FAILURE: Found {len(fk_errors)} referential integrity errors.")
        print(" ! ABORTING BACKUP TO PREVENT CORRUPTION EXPORT.")
        conn.close()
        return

    # ---------------------------------------------------------
    # STEP 3: EXPORT / BACKUP EXECUTION
    # ---------------------------------------------------------
    print(f"\n--- PHASE 3: EXECUTING BACKUP TO '{BACKUP_FILE}' ---")
    
    try:
        line_counter = 0
        with open(BACKUP_FILE, 'w', encoding='utf-8') as f:
            # Write a timestamp header
            f.write(f"-- BACKUP GENERATED: {datetime.datetime.now()}\n")
            f.write(f"-- SOURCE DB: {DB_NAME}\n\n")
            
            # Dump the data
            for line in conn.iterdump():
                f.write('%s\n' % line)
                line_counter += 1
        
        # Verify file size
        file_size_kb = os.path.getsize(BACKUP_FILE) / 1024
        
        print(f" > SQL Dump written successfully.")
        print(f" > Total Lines Written: {line_counter}")
        print(f" > File Size: {file_size_kb:.2f} KB")
        print("\n--- STATUS: COMPLETE SUCCESS ---")
        
    except Exception as e:
        print(f"\n! BACKUP FAILED: {str(e)}")
    finally:
        conn.close()

if __name__ == "__main__":
    validate_and_backup()