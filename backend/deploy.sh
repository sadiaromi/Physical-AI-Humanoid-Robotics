#!/bin/bash
# Production Deployment Script for RAG Chatbot

set -e  # Exit on any error

# Configuration
SERVICE_NAME="rag-chatbot"
IMAGE_NAME="rag-chatbot:latest"
ENV_FILE=".env.production"
CONTAINER_NAME="${SERVICE_NAME}-container"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if a command exists
command_exists() {
    command -v "$1" >/dev/null 2>&1
}

# Check prerequisites
check_prerequisites() {
    print_status "Checking prerequisites..."

    if ! command_exists docker; then
        print_error "Docker is not installed. Please install Docker and try again."
        exit 1
    fi

    if ! command_exists docker-compose; then
        print_error "Docker Compose is not installed. Please install Docker Compose and try again."
        exit 1
    fi

    print_status "Prerequisites check passed."
}

# Load environment variables
load_environment() {
    if [ -f "$ENV_FILE" ]; then
        print_status "Loading environment variables from $ENV_FILE"
        export $(cat $ENV_FILE | xargs)
    else
        print_error "$ENV_FILE not found. Please create it with required environment variables."
        exit 1
    fi
}

# Build the Docker image
build_image() {
    print_status "Building Docker image: $IMAGE_NAME"
    docker build -t $IMAGE_NAME .
    if [ $? -eq 0 ]; then
        print_status "Docker image built successfully."
    else
        print_error "Failed to build Docker image."
        exit 1
    fi
}

# Stop existing containers
stop_containers() {
    if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        print_status "Stopping existing container..."
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    elif [ "$(docker ps -aq -f status=exited -f name=$CONTAINER_NAME)" ]; then
        print_status "Removing stopped container..."
        docker rm $CONTAINER_NAME
    fi
}

# Deploy the service
deploy_service() {
    print_status "Deploying $SERVICE_NAME..."
    
    # Run the container in detached mode
    docker run -d \
        --name $CONTAINER_NAME \
        --env-file $ENV_FILE \
        -p 8000:8000 \
        --restart unless-stopped \
        $IMAGE_NAME
    
    if [ $? -eq 0 ]; then
        print_status "$SERVICE_NAME deployed successfully."
    else
        print_error "Failed to deploy $SERVICE_NAME."
        exit 1
    fi
}

# Verify deployment
verify_deployment() {
    print_status "Verifying deployment..."
    
    # Wait a few seconds for the service to start
    sleep 10
    
    # Check if the container is running
    if [ "$(docker ps -q -f name=$CONTAINER_NAME -f status=running)" ]; then
        print_status "$SERVICE_NAME is running."
    else
        print_error "$SERVICE_NAME is not running. Check logs with: docker logs $CONTAINER_NAME"
        exit 1
    fi
    
    # Check if the service is responding (if health check is available)
    # For this example, we'll just check that the container is running
    # In a more complex setup, you might want to curl a health endpoint
    print_status "Deployment verification passed."
}

# Show deployment status
show_status() {
    print_status "Service Status:"
    docker ps --filter name=$CONTAINER_NAME --format "table {{.Names}}\t{{.Status}}\t{{.Ports}}"
    
    print_status "Recent logs:"
    docker logs --tail 10 $CONTAINER_NAME
}

# Main deployment function
main() {
    print_status "Starting production deployment for $SERVICE_NAME..."
    
    check_prerequisites
    load_environment
    build_image
    stop_containers
    deploy_service
    verify_deployment
    show_status
    
    print_status "Deployment completed successfully!"
    print_status "Service is available at: http://localhost:8000"
    print_status "API documentation: http://localhost:8000/docs"
}

# Rollback function
rollback() {
    print_warning "Rolling back deployment..."
    
    if [ "$(docker ps -q -f name=$CONTAINER_NAME)" ]; then
        docker stop $CONTAINER_NAME
        docker rm $CONTAINER_NAME
    fi
    
    print_status "Rollback completed."
}

# Parse command line arguments
case "${1:-deploy}" in
    deploy)
        main
        ;;
    rollback)
        rollback
        ;;
    status)
        show_status
        ;;
    *)
        echo "Usage: $0 {deploy|rollback|status}"
        exit 1
        ;;
esac