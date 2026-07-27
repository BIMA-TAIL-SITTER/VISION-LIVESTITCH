#!/bin/bash
// filepath: git_sync.sh

set -e

echo "=== Git Sync Menu ==="
echo "1. Check for remote changes"
echo "2. Pull latest changes"
echo "3. Commit and push changes"
echo "4. Full sync (pull then commit and push)"
echo "5. Exit"
read -p "Choose an option (1-5): " OPTION

case $OPTION in
    1)
        # Check for remote changes
        read -p "Enter branch name: " BRANCH
        
        if ! git show-ref --verify --quiet refs/heads/"$BRANCH"; then
            echo "Branch '$BRANCH' does not exist locally."
            exit 1
        fi
        
        git fetch origin "$BRANCH" --quiet
        LOCAL_COMMIT=$(git rev-parse "$BRANCH")
        REMOTE_COMMIT=$(git rev-parse "origin/$BRANCH")
        BASE_COMMIT=$(git merge-base "$BRANCH" "origin/$BRANCH")
        
        if [ "$REMOTE_COMMIT" = "$BASE_COMMIT" ]; then
            echo "✓ Remote is up to date with local branch '$BRANCH'."
        elif [ "$LOCAL_COMMIT" = "$BASE_COMMIT" ]; then
            echo "⚠ Remote is ahead of local branch '$BRANCH'."
            echo "Run option 2 to pull changes."
        else
            echo "⚠ Branches have diverged. Manual merge/rebase required."
        fi
        ;;
        
    2)
        # Pull latest changes
        read -p "Enter branch name: " BRANCH
        
        if ! git show-ref --verify --quiet refs/heads/"$BRANCH"; then
            echo "Branch '$BRANCH' does not exist locally."
            exit 1
        fi
        
        git checkout --quiet "$BRANCH"
        
        if git pull --rebase origin "$BRANCH"; then
            echo "✓ Successfully pulled latest changes from '$BRANCH'."
        else
            echo "✗ Failed to pull changes. You may need to resolve conflicts."
            exit 1
        fi
        ;;
        
    3)
        # Commit and push
        read -p "Enter branch name: " BRANCH
        
        if git show-ref --verify --quiet refs/heads/"$BRANCH"; then
            git checkout --quiet "$BRANCH"
        else
            read -p "Branch '$BRANCH' not found. Create new branch? (y/n): " CREATE
            if [[ "$CREATE" =~ ^[Yy]$ ]]; then
                git checkout -b "$BRANCH"
                echo "✓ Created and switched to branch '$BRANCH'."
            else
                exit 1
            fi
        fi
        
        # Check for changes
        if git diff-index --quiet HEAD --; then
            echo "No changes to commit."
            exit 0
        fi
        
        read -p "Enter commit message: " COMMIT_MSG
        if [ -z "$COMMIT_MSG" ]; then
            echo "Commit message cannot be empty."
            exit 1
        fi
        
        git add .
        git commit -m "$COMMIT_MSG"
        
        # Check remote status before pushing
        git fetch origin "$BRANCH" --quiet 2>/dev/null || true
        
        if git push origin "$BRANCH"; then
            echo "✓ Successfully pushed changes to '$BRANCH'."
        else
            echo "✗ Push failed. Remote may be ahead of local."
            echo "Run option 4 for full sync or manually resolve."
            exit 1
        fi
        ;;
        
    4)
        # Full sync
        read -p "Enter branch name: " BRANCH
        
        if git show-ref --verify --quiet refs/heads/"$BRANCH"; then
            git checkout --quiet "$BRANCH"
        else
            read -p "Branch '$BRANCH' not found. Create new branch? (y/n): " CREATE
            if [[ "$CREATE" =~ ^[Yy]$ ]]; then
                git checkout -b "$BRANCH"
                echo "✓ Created and switched to branch '$BRANCH'."
            else
                exit 1
            fi
        fi
        
        # Fetch and check remote
        echo "Checking remote status..."
        git fetch origin "$BRANCH" --quiet 2>/dev/null || echo "Note: Remote branch may not exist yet"
        
        LOCAL_COMMIT=$(git rev-parse "$BRANCH" 2>/dev/null || echo "")
        REMOTE_COMMIT=$(git rev-parse "origin/$BRANCH" 2>/dev/null || echo "")
        
        if [ -n "$REMOTE_COMMIT" ] && [ -n "$LOCAL_COMMIT" ]; then
            BASE_COMMIT=$(git merge-base "$BRANCH" "origin/$BRANCH")
            
            if [ "$LOCAL_COMMIT" != "$BASE_COMMIT" ] && [ "$REMOTE_COMMIT" != "$BASE_COMMIT" ]; then
                echo "⚠ Branches have diverged. Manual intervention required."
                exit 1
            elif [ "$LOCAL_COMMIT" != "$REMOTE_COMMIT" ]; then
                echo "Pulling latest changes..."
                if git pull --rebase origin "$BRANCH"; then
                    echo "✓ Successfully pulled changes."
                else
                    echo "✗ Pull failed. Resolve conflicts and try again."
                    exit 1
                fi
            fi
        fi
        
        # Check for local changes
        if git diff-index --quiet HEAD --; then
            echo "No local changes to commit."
            exit 0
        fi
        
        read -p "Enter commit message: " COMMIT_MSG
        if [ -z "$COMMIT_MSG" ]; then
            echo "Commit message cannot be empty."
            exit 1
        fi
        
        git add .
        git commit -m "$COMMIT_MSG"
        
        if git push origin "$BRANCH"; then
            echo "✓ Successfully completed full sync for '$BRANCH'."
        else
            echo "✗ Push failed."
            exit 1
        fi
        ;;
        
    5)
        echo "Exiting."
        exit 0
        ;;
        
    *)
        echo "Invalid option. Exiting."
        exit 1
        ;;
esac