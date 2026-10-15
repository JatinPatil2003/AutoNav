FROM node:20-alpine as build

WORKDIR /app

COPY ./PlatformApp/package*.json ./

RUN npm install

COPY ./PlatformApp .

RUN npm run build

# Stage 2: Serve the application using Nginx
FROM nginx:alpine

# Copy the build files from the previous stage
COPY --from=build /app/dist /usr/share/nginx/html

# Copy Nginx configuration file
COPY ./PlatformApp/nginx.conf /etc/nginx/conf.d/default.conf

# Expose port 80
EXPOSE 80

# Start Nginx
CMD ["nginx", "-g", "daemon off;"]