FROM node:20-alpine AS build
WORKDIR /app
COPY ./PlatformApp/package*.json ./
RUN npm install
COPY ./PlatformApp .
RUN npm run build

FROM nginx:alpine
COPY --from=build /app/dist /usr/share/nginx/html
COPY ./PlatformApp/nginx.conf /etc/nginx/conf.d/default.conf
EXPOSE 80
CMD ["nginx", "-g", "daemon off;"]
