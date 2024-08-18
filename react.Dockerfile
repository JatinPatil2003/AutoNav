FROM node:20-alpine as build

WORKDIR /app

COPY ./ReactApp/package*.json ./

RUN npm install

COPY ./ReactApp .

RUN npm run build

# Stage 2: Serve the application using Nginx
FROM nginx:alpine

# Copy the build files from the previous stage
COPY --from=build /app/build /usr/share/nginx/html

# Copy Nginx configuration file
COPY ./ReactApp/nginx.conf /etc/nginx/conf.d/default.conf

# Expose port 80
EXPOSE 80

# Start Nginx
CMD ["nginx", "-g", "daemon off;"]